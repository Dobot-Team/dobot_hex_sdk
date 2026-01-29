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
# Program version information
# ----------------------------
PROGRAM_MAJOR_VERSION = 1
PROGRAM_MINOR_VERSION = 0


class RobotStateEnum(IntEnum):
    """Enumeration of robot states used by the control logic."""

    PASSIVE = 0
    STAND_DOWN = 1
    STAND_UP = 2
    BALANCE_STAND = 3
    WALK = 4


def arr_to_str(a, prec: int = 3):
    """
    Convert a sequence (list/tuple/numpy array) to a fixed-precision string.

    Formats values with a fixed number of decimals and wraps as "[x, y, z]".

    Args:
        a: Iterable of numeric values (list/tuple/numpy array).
        prec: Number of decimal places.

    Returns:
        str: Formatted string, e.g. "[1.000, 2.000, 3.000]".
    """
    fmt = f"{{:.{prec}f}}"
    return "[" + ", ".join(fmt.format(float(x)) for x in a) + "]"


class robotStateFuncNode(Node):
    """
    ROS2 node for robot state monitoring.

    Responsibilities:
    - Subscribe to /robot_state and cache the latest RobotState message.
    - Periodically (50 Hz) attempt state progression via /robot_cmd (optional).
    - Print a fixed-width, in-place refreshed table of primary state fields.
    """

    def __init__(self):
        """Initialize publishers, subscribers, timer, and table settings."""
        super().__init__("robot_state_subscriber")

        # Topic names
        self.robot_command_topic_name = "/robot_cmd"
        self.robot_state_topic_name = "/robot_state"

        # Timer (20 ms, ~50 Hz)
        self.publisher_timer = self.create_timer(0.02, self.robot_command_contrl_msg)

        # Publishers / Subscribers
        self.robot_command_publisher = self.create_publisher(
            RobotCommand, self.robot_command_topic_name, 1
        )
        self.robot_state_subscriber = self.create_subscription(
            RobotState, self.robot_state_topic_name, self.robot_state_parse, 1
        )

        self.robot_state_curr: int = 0
        self.last_print_time = None  # rclpy Time
        self.printed_once = False  # whether a table has been printed already
        self.msg_robot_state = RobotState()  # last received RobotState

        # Table layout (mirrors C++ fixed-width formatting)
        self.w_name = 13
        self.w_value = 36
        # lines: top sep(1) + header(1) + mid sep(1) + 5 data rows + bottom sep(1) = 9
        self.table_lines = 9

    def robot_command_contrl_msg(self):
        """
        Periodic timer callback to (optionally) advance robot state.

        Behavior:
        - If current state < STAND_DOWN, publish a RobotCommand to move forward one state.
        - Otherwise, do nothing (user controls robot externally and watches the console).
        """
        if self.robot_state_curr < RobotStateEnum.STAND_DOWN:
            target = int(self.robot_state_curr) + 1
            cmd = RobotCommand()
            cmd.target_state = target
            self.robot_command_publisher.publish(cmd)
            time.sleep(1)  # allow time for state transition
        return

    def robot_state_parse(self, msg):
        """
        Subscriber callback for /robot_state.

        Updates cached state and prints a formatted table in place (console),
        throttled to at most 10 Hz (every 0.1 s).

        Args:
            msg: The latest RobotState message.
        """
        # Cache state
        self.robot_state_curr = msg.temp[10]
        self.msg_robot_state = msg

        # Throttle printing to 10 Hz
        now = self.get_clock().now()
        if self.last_print_time is not None:
            if (now - self.last_print_time).nanoseconds < int(0.1 * 1e9):
                return
        self.last_print_time = now

        # Move cursor up to overwrite previous table (ANSI escape)
        if self.printed_once:
            print(f"\x1b[{self.table_lines}A\r", end="")
        else:
            self.printed_once = True
            print()  # leading blank line on first print for readability

        # Extract and format fields
        pos_body = arr_to_str(msg.pos_body, prec=3)
        vel_body = arr_to_str(msg.vel_body, prec=3)
        acc_body = arr_to_str(msg.acc_body, prec=3)
        ori_body = arr_to_str(msg.ori_body, prec=3)  # quaternion [x, y, z, w]
        omega_body = arr_to_str(msg.omega_body, prec=3)

        # Helpers to print a table row and separators
        def line_sep():
            print("+" + "-" * (self.w_name + 2) + "+" + "-" * (self.w_value + 1) + "+")

        def print_row(name: str, val: str):
            print(f"| {name:<{self.w_name}} | {val:<{self.w_value}}|")

        # Render the table
        line_sep()
        print_row("Name", "Value")
        line_sep()
        print_row("pos_body", pos_body)
        print_row("vel_body", vel_body)
        print_row("acc_body", acc_body)
        print_row("ori_body", ori_body)
        print_row("omega_body", omega_body)
        line_sep()

        # Force immediate flush
        print("", end="", flush=True)


def main(args=None):
    """Main entry point for ROS2 node execution."""
    rclpy.init(args=args)
    robot_control_node = robotStateFuncNode()
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
