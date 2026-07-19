#!/usr/bin/env python3
"""记录 AirSim ENU 真值里程计，严禁把该 Topic 输入 VINS-Fusion。"""

import threading
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry

from slam_practice.trajectory import format_tum_pose


class AirSimGroundTruthRecorder:
    def __init__(self) -> None:
        input_topic = rospy.get_param("~input_topic", "/airsim_node/Drone1/odom_local_enu")
        output_path = Path(rospy.get_param("~output_path")).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = output_path.open("w", encoding="utf-8", buffering=1)
        self.lock = threading.Lock()
        self.last_stamp = -1.0
        self.subscriber = rospy.Subscriber(input_topic, Odometry, self.callback, queue_size=200)
        rospy.on_shutdown(self.close)
        rospy.loginfo("Recording AirSim truth from %s to %s", input_topic, output_path)

    def callback(self, message: Odometry) -> None:
        timestamp = message.header.stamp.to_sec()
        pose = message.pose.pose
        with self.lock:
            if timestamp <= self.last_stamp:
                return
            self.handle.write(
                format_tum_pose(
                    timestamp,
                    (pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                ) + "\n"
            )
            self.last_stamp = timestamp

    def close(self) -> None:
        with self.lock:
            if not self.handle.closed:
                self.handle.flush()
                self.handle.close()


def main() -> None:
    rospy.init_node("airsim_gt_recorder")
    AirSimGroundTruthRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
