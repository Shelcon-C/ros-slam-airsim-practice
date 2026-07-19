cmake_minimum_required(VERSION 3.0.2)
project(slam_practice)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/euroc_mono_publisher.py
  scripts/trajectory_recorder.py
  scripts/euroc_groundtruth_to_tum.py
  scripts/calibrate_camera.py
  scripts/evaluate_trajectory.py
  scripts/stereo_imu_relay.py
  scripts/vins_output_adapter.py
  scripts/airsim_gt_recorder.py
  scripts/depth_pose_adapter.py
  scripts/ego_position_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
