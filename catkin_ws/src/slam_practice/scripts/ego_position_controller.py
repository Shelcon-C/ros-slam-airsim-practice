#!/usr/bin/env python3
"""将 EGO PositionCommand 转换为带看门狗的 AirSim 世界系速度命令。"""

import threading

import numpy as np
import rospy
from airsim_ros_pkgs.msg import VelCmd
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_srvs.srv import SetBool, SetBoolResponse

from slam_practice.control import compute_velocity_command


class EgoPositionController:
    def __init__(self) -> None:
        odometry_topic = rospy.get_param("~odometry_topic", "/slam_practice/vins/odometry")
        command_topic = rospy.get_param("~command_topic", "/planning/pos_cmd")
        output_topic = rospy.get_param("~output_topic", "/airsim_node/vel_cmd_world_frame")
        self.kp = float(rospy.get_param("~kp", 0.8))
        self.max_xy_speed = float(rospy.get_param("~max_xy_speed", 1.5))
        self.max_z_speed = float(rospy.get_param("~max_z_speed", 0.8))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 1.0))
        self.watchdog_timeout = float(rospy.get_param("~watchdog_timeout", 0.25))
        control_rate = float(rospy.get_param("~control_rate", 30.0))
        if self.watchdog_timeout <= 0 or control_rate <= 0 or self.max_yaw_rate <= 0:
            raise ValueError("watchdog, rate and yaw limit must be positive")

        self.enabled = False
        self.lock = threading.Lock()
        self.current_position = None
        self.target_position = None
        self.feedforward_velocity = None
        self.yaw_rate = 0.0
        self.last_odometry_receipt = None
        self.last_command_receipt = None
        self.publisher = rospy.Publisher(output_topic, VelCmd, queue_size=10)
        self.odometry_subscriber = rospy.Subscriber(
            odometry_topic, Odometry, self.odometry_callback, queue_size=30
        )
        self.command_subscriber = rospy.Subscriber(
            command_topic, PositionCommand, self.command_callback, queue_size=30
        )
        self.enable_service = rospy.Service("~set_enabled", SetBool, self.enable_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0 / control_rate), self.timer_callback)
        rospy.on_shutdown(self.stop)
        rospy.logwarn("EGO AirSim controller starts DISABLED; call ~set_enabled to unlock")

    def odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        with self.lock:
            self.current_position = np.array((position.x, position.y, position.z), dtype=float)
            self.last_odometry_receipt = rospy.Time.now()

    def command_callback(self, message: PositionCommand) -> None:
        with self.lock:
            self.target_position = np.array(
                (message.position.x, message.position.y, message.position.z), dtype=float
            )
            self.feedforward_velocity = np.array(
                (message.velocity.x, message.velocity.y, message.velocity.z), dtype=float
            )
            self.yaw_rate = float(message.yaw_dot)
            self.last_command_receipt = rospy.Time.now()

    def enable_callback(self, request) -> SetBoolResponse:
        with self.lock:
            self.enabled = bool(request.data)
        if not request.data:
            self.publish_velocity(np.zeros(3), 0.0)
        state = "enabled" if request.data else "disabled and stopped"
        rospy.logwarn("EGO AirSim controller %s", state)
        return SetBoolResponse(success=True, message=state)

    def timer_callback(self, _event) -> None:
        now = rospy.Time.now()
        with self.lock:
            enabled = self.enabled
            current = None if self.current_position is None else self.current_position.copy()
            target = None if self.target_position is None else self.target_position.copy()
            feedforward = (
                None if self.feedforward_velocity is None else self.feedforward_velocity.copy()
            )
            yaw_rate = self.yaw_rate
            odometry_time = self.last_odometry_receipt
            command_time = self.last_command_receipt
        inputs_ready = all(value is not None for value in (current, target, feedforward))
        fresh = (
            odometry_time is not None
            and command_time is not None
            and (now - odometry_time).to_sec() <= self.watchdog_timeout
            and (now - command_time).to_sec() <= self.watchdog_timeout
        )
        if not enabled or not inputs_ready or not fresh:
            self.publish_velocity(np.zeros(3), 0.0)
            return
        try:
            velocity = compute_velocity_command(
                current, target, feedforward, self.kp, self.max_xy_speed, self.max_z_speed
            )
        except ValueError as error:
            rospy.logerr_throttle(1.0, "Invalid controller input: %s", error)
            self.publish_velocity(np.zeros(3), 0.0)
            return
        self.publish_velocity(velocity, np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate))

    def publish_velocity(self, velocity, yaw_rate: float) -> None:
        message = VelCmd()
        message.twist.linear.x = float(velocity[0])
        message.twist.linear.y = float(velocity[1])
        message.twist.linear.z = float(velocity[2])
        message.twist.angular.z = float(yaw_rate)
        self.publisher.publish(message)

    def stop(self) -> None:
        self.publish_velocity(np.zeros(3), 0.0)


def main() -> None:
    rospy.init_node("ego_position_controller")
    EgoPositionController()
    rospy.spin()


if __name__ == "__main__":
    main()
