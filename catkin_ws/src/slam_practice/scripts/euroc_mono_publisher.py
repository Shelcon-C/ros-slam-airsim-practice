#!/usr/bin/env python3
"""按 EuRoC 原始时间间隔发布左目图像和相机信息。"""

from pathlib import Path
import time

import cv2
import rospy
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from slam_practice.dataset import load_euroc_camera_index


def main() -> None:
    rospy.init_node("euroc_mono_publisher")
    dataset_root = Path(rospy.get_param("~dataset_root")).expanduser()
    camera_yaml = Path(rospy.get_param("~camera_yaml")).expanduser().resolve()
    playback_rate = float(rospy.get_param("~playback_rate", 1.0))
    frame_id = rospy.get_param("~frame_id", "camera_mono_optical_frame")
    image_topic = rospy.get_param("~image_topic", "/camera/mono/image_raw")
    info_topic = rospy.get_param("~camera_info_topic", "/camera/mono/camera_info")
    start_offset = max(0.0, float(rospy.get_param("~start_offset", 0.0)))
    duration = float(rospy.get_param("~duration", 0.0))
    wait_for_subscriber = bool(rospy.get_param("~wait_for_subscriber", True))
    subscriber_timeout = float(rospy.get_param("~subscriber_timeout", 60.0))
    if playback_rate <= 0:
        raise ValueError("~playback_rate must be positive")
    if subscriber_timeout < 0:
        raise ValueError("~subscriber_timeout must be non-negative")

    frames = load_euroc_camera_index(dataset_root)
    first_time = frames[0].timestamp_sec
    selected = [
        frame for frame in frames
        if frame.timestamp_sec - first_time >= start_offset
        and (duration <= 0 or frame.timestamp_sec - first_time <= start_offset + duration)
    ]
    if not selected:
        raise ValueError("No EuRoC frames remain after start_offset/duration filtering")

    manager = CameraInfoManager(cname="euroc_cam0", url=f"file://{camera_yaml}")
    if not manager.loadCameraInfo():
        raise RuntimeError(f"Unable to load camera calibration: {camera_yaml}")

    bridge = CvBridge()
    image_publisher = rospy.Publisher(image_topic, Image, queue_size=5)
    info_publisher = rospy.Publisher(info_topic, CameraInfo, queue_size=5)
    if wait_for_subscriber:
        rospy.loginfo("Waiting for the ORB-SLAM3 image subscriber before playback")
        deadline = time.monotonic() + subscriber_timeout if subscriber_timeout > 0 else None
        while not rospy.is_shutdown() and image_publisher.get_num_connections() == 0:
            if deadline is not None and time.monotonic() >= deadline:
                raise RuntimeError(
                    f"No subscriber connected to {image_topic} within {subscriber_timeout:.1f} seconds"
                )
            rospy.sleep(0.1)
    rospy.loginfo("Publishing %d EuRoC mono frames at %.2fx", len(selected), playback_rate)

    previous_timestamp = None
    for frame in selected:
        if rospy.is_shutdown():
            break
        if previous_timestamp is not None:
            rospy.sleep(max(0.0, (frame.timestamp_sec - previous_timestamp) / playback_rate))
        image = cv2.imread(str(frame.path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"OpenCV failed to read image: {frame.path}")

        stamp = rospy.Time.from_sec(frame.timestamp_sec)
        image_message = bridge.cv2_to_imgmsg(image, encoding="mono8")
        image_message.header.stamp = stamp
        image_message.header.frame_id = frame_id
        info_message = manager.getCameraInfo()
        info_message.header.stamp = stamp
        info_message.header.frame_id = frame_id
        image_publisher.publish(image_message)
        info_publisher.publish(info_message)
        previous_timestamp = frame.timestamp_sec

    rospy.loginfo("EuRoC sequence publishing finished")


if __name__ == "__main__":
    main()
