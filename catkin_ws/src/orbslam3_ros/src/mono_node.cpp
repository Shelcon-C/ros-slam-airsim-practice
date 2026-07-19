// ORB-SLAM3 单目 ROS Wrapper。
//
// ORB-SLAM3 TrackMonocular 返回 Tcw，即世界坐标到相机坐标的变换。
// ROS 中发布相机在世界坐标中的位姿，因此必须先求逆得到 Twc。

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <MapPoint.h>
#include <System.h>
#include <Tracking.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_ros/transform_broadcaster.h>


class MonoNode {
 public:
  MonoNode() : private_node_("~") {
    std::string vocabulary_path;
    std::string settings_path;
    std::string image_topic;
    private_node_.param<std::string>("vocabulary_path", vocabulary_path, "");
    private_node_.param<std::string>("settings_path", settings_path, "");
    private_node_.param<std::string>("image_topic", image_topic, "/camera/mono/image_raw");
    private_node_.param<std::string>("world_frame", world_frame_, "map");
    private_node_.param<std::string>("camera_frame", camera_frame_, "camera_mono_optical_frame");
    private_node_.param<std::string>("keyframe_trajectory", keyframe_trajectory_, "keyframes.tum");
    private_node_.param<int>("max_path_poses", max_path_poses_, 20000);
    private_node_.param<bool>("use_viewer", use_viewer_, true);
    if (vocabulary_path.empty() || settings_path.empty()) {
      throw std::invalid_argument("~vocabulary_path and ~settings_path are required");
    }
    if (max_path_poses_ <= 0) {
      throw std::invalid_argument("~max_path_poses must be positive");
    }

    slam_ = std::make_unique<ORB_SLAM3::System>(
        vocabulary_path, settings_path, ORB_SLAM3::System::MONOCULAR, use_viewer_);
    pose_publisher_ = node_.advertise<geometry_msgs::PoseStamped>("/orbslam3/pose", 10);
    odometry_publisher_ = node_.advertise<nav_msgs::Odometry>("/orbslam3/odometry", 10);
    path_publisher_ = node_.advertise<nav_msgs::Path>("/orbslam3/path", 2, true);
    points_publisher_ = node_.advertise<sensor_msgs::PointCloud>("/orbslam3/tracked_points", 2);
    image_subscriber_ = node_.subscribe(image_topic, 10, &MonoNode::imageCallback, this);
    path_.header.frame_id = world_frame_;
    ROS_INFO_STREAM("ORB-SLAM3 mono wrapper subscribed to " << image_topic);
  }

  ~MonoNode() {
    if (slam_) {
      slam_->Shutdown();
      // 单目模式不能使用 SaveTrajectoryTUM，只保存所有关键帧轨迹。
      slam_->SaveKeyFrameTrajectoryTUM(keyframe_trajectory_);
      ROS_INFO_STREAM("Saved ORB-SLAM3 keyframe trajectory to " << keyframe_trajectory_);
    }
  }

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    cv_bridge::CvImageConstPtr image;
    try {
      image = cv_bridge::toCvShare(message);
    } catch (const cv_bridge::Exception& error) {
      ROS_ERROR_STREAM_THROTTLE(2.0, "cv_bridge conversion failed: " << error.what());
      return;
    }

    const double timestamp = message->header.stamp.toSec();
    const Sophus::SE3f Tcw = slam_->TrackMonocular(image->image, timestamp);
    if (slam_->GetTrackingState() != ORB_SLAM3::Tracking::OK) {
      ROS_WARN_THROTTLE(2.0, "ORB-SLAM3 has not produced a valid tracked pose yet");
      return;
    }
    const Sophus::SE3f Twc = Tcw.inverse();
    publishPose(Twc, message->header.stamp);
    publishTrackedPoints(message->header.stamp);
  }

  void publishPose(const Sophus::SE3f& Twc, const ros::Time& stamp) {
    const Eigen::Vector3f translation = Twc.translation();
    Eigen::Quaternionf quaternion(Twc.rotationMatrix());
    quaternion.normalize();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    pose_publisher_.publish(pose);

    nav_msgs::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = camera_frame_;
    odometry.pose.pose = pose.pose;
    odometry_publisher_.publish(odometry);

    path_.header.stamp = stamp;
    path_.poses.push_back(pose);
    if (static_cast<int>(path_.poses.size()) > max_path_poses_) {
      const auto remove_count = path_.poses.size() - static_cast<std::size_t>(max_path_poses_);
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + remove_count);
    }
    path_publisher_.publish(path_);

    geometry_msgs::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = camera_frame_;
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation = pose.pose.orientation;
    transform_broadcaster_.sendTransform(transform);
  }

  void publishTrackedPoints(const ros::Time& stamp) {
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = world_frame_;
    for (ORB_SLAM3::MapPoint* map_point : slam_->GetTrackedMapPoints()) {
      if (map_point == nullptr || map_point->isBad()) {
        continue;
      }
      const Eigen::Vector3f position = map_point->GetWorldPos();
      geometry_msgs::Point32 point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      cloud.points.push_back(point);
    }
    points_publisher_.publish(cloud);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber image_subscriber_;
  ros::Publisher pose_publisher_;
  ros::Publisher odometry_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher points_publisher_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  nav_msgs::Path path_;
  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string keyframe_trajectory_;
  int max_path_poses_;
  bool use_viewer_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "orbslam3_mono_node");
  try {
    MonoNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("ORB-SLAM3 wrapper failed: " << error.what());
    return 1;
  }
  return 0;
}
