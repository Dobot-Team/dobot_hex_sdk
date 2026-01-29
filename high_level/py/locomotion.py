"""
This file is Dobot Robotics's property. It contains Dobot Robotics trade secret, proprietary and confidential information.

The information and code contained in this file is only for authorized Dobot Robotics employees to design, create, modify, or review.

DO NOT DISTRIBUTE, DO NOT DUPLICATE OR TRANSMIT IN ANY FORM WITHOUT PROPER AUTHORIZATION.

If you are not an intended recipient of this file, you must not copy, distribute, modify, or take any action in reliance on it.

If you have received this file in error, please immediately notify Dobot Robotics and permanently delete the original and any copy of any file and any printout thereof.

Copyright (c) 2025 Dobot Robotics


Author: Yixuan Chen
Mail: chenyixuan@dobot-robots.com
Created on Thu Sep 25 2025
"""

import numpy as np
import time
from enum import IntEnum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from custom_msg.msg import RobotCommand
from custom_msg.msg import RobotState

# ----------------------------
# Program version information
# ----------------------------
PROGRAM_MAJOR_VERSION = 1
PROGRAM_MINOR_VERSION = 0


class RobotStateEnum(IntEnum):
    """Enumeration of robot states used in control logic."""

    PASSIVE = 0
    STAND_DOWN = 1
    STAND_UP = 2
    BALANCE_STAND = 3
    WALK = 4


class locomotionFuncNode(Node):
    """
    ROS2 node for locomotion control testing.

    Responsibilities:
    - Subscribe to /robot_state to get current robot state
    - Publish RobotCommand to switch robot modes
    - Publish Twist velocity messages to /vel_cmd in walking state
    """

    def __init__(self):
        """Initialize publishers, subscribers, and periodic timer."""
        super().__init__("locomotion_control_publisher")

        # Topic names
        self.robot_command_topic_name = "/robot_cmd"
        self.robot_state_topic_name = "/robot_state"
        self.vel_cmd_topic_name = "/vel_cmd"

        # Timer: run every 20 ms (~50 Hz)
        self.publisher_timer = self.create_timer(0.02, self.publish_robot_control_msg)
        # Publishers
        self.vel_cmd_publisher = self.create_publisher(
            Twist, self.vel_cmd_topic_name, 1
        )
        self.robot_command_publisher = self.create_publisher(
            RobotCommand, self.robot_command_topic_name, 1
        )
        # Subscriber
        self.robot_state_subscriber = self.create_subscription(
            RobotState, self.robot_state_topic_name, self.robot_state_parse, 1
        )

        self.robot_control_msg = Twist()
        self.robot_state_curr: int = 0

        self.first_run : bool = True

    def publish_robot_control_msg(self):
        """
        Main control callback.

        Behavior:
        - If current state < WALK: publish RobotCommand to step state forward
        - If in WALK: publish a sequence of velocity commands
          (forward → backward → left → right → yaw left → yaw right)
          Each segment lasts T seconds
        """
        if self.first_run and self.robot_state_curr < RobotStateEnum.WALK:
            # Step state machine toward WALK
            target = int(self.robot_state_curr) + 1
            cmd = RobotCommand()
            cmd.target_state = target
            self.robot_command_publisher.publish(cmd)
            time.sleep(1)  # allow state transition
        else:
            self.first_run = False
            # Initialize timer once
            if not hasattr(self, "t0"):
                self.t0 = self.get_clock().now()
            if not hasattr(self, "T"):
                self.T = 1.5  # 每段持续时间 (秒)

            # Elapsed time since sequence start
            dt = (self.get_clock().now() - self.t0).nanoseconds / 1e9

            # Motion profiles
            v_forward = +0.4
            v_backward = -0.4
            v_left = +0.4
            v_right = -0.4
            v_left_yaw = +0.8
            v_right_yaw = -0.8

            # Default speeds
            fXSpeed = 0.0
            fYSpeed = 0.0
            fYawSpeed = 0.0

            # Motion sequence state machine
            if dt < 1 * self.T:
                fXSpeed = v_forward
            elif dt < 2 * self.T:
                fXSpeed = v_backward
            elif dt < 3 * self.T:
                fYSpeed = v_left
            elif dt < 4 * self.T:
                fYSpeed = v_right
            elif dt < 5 * self.T:
                fYawSpeed = v_left_yaw
            elif dt < 6 * self.T:
                fYawSpeed = v_right_yaw
            else:
                if self.robot_state_curr > RobotStateEnum.PASSIVE:
                    if self.robot_state_curr == RobotStateEnum.BALANCE_STAND:
                        target = int(self.robot_state_curr) - 2
                    else:
                        target = int(self.robot_state_curr) - 1
                    cmd = RobotCommand()
                    cmd.target_state = target
                    self.robot_command_publisher.publish(cmd)
                    time.sleep(1)  # allow state transition

            # Fill velocity command
            controlMsg = Twist()
            controlMsg.linear.x = fXSpeed * 0.5
            controlMsg.linear.y = fYSpeed * 0.4
            controlMsg.angular.z = fYawSpeed * 0.35

            self.vel_cmd_publisher.publish(controlMsg)

    def robot_state_parse(self, msg):
        """
        Robot state subscriber callback.
        Updates current robot state from /robot_state.
        """
        self.robot_state_curr = msg.temp[10]
        self.get_logger().info(f"robot state: {self.robot_state_curr}")


def main(args=None):
    """Main entry point for ROS2 node execution."""
    rclpy.init(args=args)
    robot_control_node = locomotionFuncNode()
    robot_control_node.get_logger().info("Starting Python publisher node...")

    try:
        rclpy.spin(robot_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
