#!/usr/bin/env python3
"""把 PoseStamped 或 Odometry 以 TUM 格式持续写入文件。"""

import threading
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from slam_practice.trajectory import format_tum_pose


class TrajectoryRecorder:
    def __init__(self) -> None:
        self.output_path = Path(rospy.get_param("~output_path")).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.output_path.open("w", encoding="utf-8", buffering=1)
        self.lock = threading.Lock()
        self.last_stamp = -1.0
        input_topic = rospy.get_param("~input_topic", "/orbslam3/pose")
        message_type = rospy.get_param("~message_type", "pose").lower()
        if message_type == "pose":
            message_class = PoseStamped
        elif message_type == "odometry":
            message_class = Odometry
        else:
            raise ValueError("~message_type must be 'pose' or 'odometry'")
        self.subscriber = rospy.Subscriber(input_topic, message_class, self.callback, queue_size=100)
        rospy.on_shutdown(self.close)
        rospy.loginfo("Recording %s (%s) to %s", input_topic, message_type, self.output_path)

    def callback(self, message) -> None:
        pose = message.pose if isinstance(message, PoseStamped) else message.pose.pose
        timestamp = message.header.stamp.to_sec()
        with self.lock:
            if timestamp <= self.last_stamp:
                rospy.logwarn_throttle(5.0, "Skipping non-monotonic trajectory timestamp")
                return
            line = format_tum_pose(
                timestamp,
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            )
            self.handle.write(line + "\n")
            self.last_stamp = timestamp

    def close(self) -> None:
        with self.lock:
            if not self.handle.closed:
                self.handle.flush()
                self.handle.close()


def main() -> None:
    rospy.init_node("trajectory_recorder")
    TrajectoryRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
