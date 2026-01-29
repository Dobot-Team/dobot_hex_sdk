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
import math
from enum import IntEnum
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Joy

from custom_msg.msg import RobotCommand
from custom_msg.msg import RobotState

# ----------------------------
# Program Version Information
# ----------------------------
PROGRAM_MAJOR_VERSION = 1
PROGRAM_MINOR_VERSION = 0


class RobotStateEnum(IntEnum):
    """Enumeration of robot states, aligned with firmware definitions"""

    PASSIVE = 0
    STAND_DOWN = 1
    STAND_UP = 2
    BALANCE_STAND = 3
    WALK = 4


class bodyTwistFuncNode(Node):
    """
    ROS2 Node for body twist control.

    Responsibilities:
    - Initialize publishers/subscribers/timers
    - Publish robot command messages to switch states
    - Publish Joy messages to simulate joystick control for testing balance
    """

    def __init__(self):
        super().__init__("body_twist_control_publisher")

        # Topic names
        self.robot_command_topic_name = "/robot_cmd"
        self.robot_state_topic_name = "/robot_state"
        self.joy_handle_topic_name = "/joy"

        # Timer (50Hz) → periodically publish control messages
        self.publisher_timer = self.create_timer(0.02, self.body_twist_control_msg)

        # Publishers
        self.robot_command_publisher = self.create_publisher(
            RobotCommand, self.robot_command_topic_name, 1
        )
        self.joyTopicPublisher = self.create_publisher(
            Joy, self.joy_handle_topic_name, 1
        )
        # Subscriber for robot state feedback
        self.robot_state_subscriber = self.create_subscription(
            RobotState, self.robot_state_topic_name, self.robot_state_parse, 1
        )

        # Internal state cache
        self.robot_state_curr: int = 0
        self.count = 0.0

    def body_twist_control_msg(self):
        """
        Main control loop.
        - If robot not yet in BALANCE_STAND, send RobotCommand to switch states
        - Otherwise publish sinusoidal Joy values to test robot's roll/pitch/yaw/z
        """
        if self.robot_state_curr < RobotStateEnum.BALANCE_STAND:
            # Step through states until balance stand is reached
            target = int(self.robot_state_curr) + 1
            cmd = RobotCommand()
            cmd.target_state = target
            self.robot_command_publisher.publish(cmd)
            time.sleep(1)  # Allow transition
        else:
            # Generate sinusoidal commands when in balance stand
            self.count = self.count + 1.0 if self.count <= 10000.0 else 0.0

            # Frequencies of motion signals (Hz)
            f_roll = 0.012
            f_pitch = 0.012
            f_yaw = 0.012  # Hz：yaw 频率（Lx）
            f_z = 0.012

            # Initialize Joy message arrays
            joyMsg = Joy()
            joyMsg.header.stamp = self.get_clock().now().to_msg()

            joyMsg.axes = [0.0] * 8
            joyMsg.buttons = [0] * 11

            # Fixed trigger activation
            joyMsg.axes[2] = 1.0
            joyMsg.axes[5] = 1.0

            # Sinusoidal motion generation
            roll_val = 0.7 * math.cos(2 * math.pi * f_roll * self.count)  # roll
            pitch_val = 0.7 * math.cos(2 * math.pi * f_z * self.count)  # pitch
            yaw_val = 0.7 * math.sin(2 * math.pi * f_yaw * self.count)  # yaw
            z_val = 0.7 * math.sin(2 * math.pi * f_pitch * self.count)  # z

            # Clamp values into [-1, 1]
            clamp = lambda x: max(-1.0, min(1.0, float(x)))
            joyMsg.axes[3] = clamp(roll_val)
            joyMsg.axes[4] = clamp(pitch_val)
            joyMsg.axes[0] = clamp(yaw_val)
            joyMsg.axes[1] = clamp(z_val)

            self.joyTopicPublisher.publish(joyMsg)

    def robot_state_parse(self, msg):
        """
        Callback function to update robot state.
        Subscribed to /robot_state.
        """
        self.robot_state_curr = msg.temp[10]
        self.get_logger().info(f"robot state: {self.robot_state_curr}")


def main(args=None):
    """Main entry point for ROS2 node execution"""
    rclpy.init(args=args)
    robot_control_node = bodyTwistFuncNode()
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
