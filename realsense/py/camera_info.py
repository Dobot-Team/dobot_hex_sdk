"""
This file is Dobot Robotics's property. It contains Dobot Robotics trade secret, proprietary and confidential information.

The information and code contained in this file is only for authorized Dobot Robotics employees to design, create, modify, or review.

DO NOT DISTRIBUTE, DO NOT DUPLICATE OR TRANSMIT IN ANY FORM WITHOUT PROPER AUTHORIZATION.

If you are not an intended recipient of this file, you must not copy, distribute, modify, or take any action in reliance on it.

If you have received this file in error, please immediately notify Dobot Robotics and permanently delete the original and any copy of any file and any printout thereof.

Copyright (c) 2025 Dobot Robotics


Author: Yixuan Chen
Mail: chenyixuan@dobot-robots.com
Created on Fri Sep 26 2025
"""

"""
1. file function
@file camera_info.py
@author  Yixuan Chen
@brief Subscribe a Realsense depth CameraInfo topic and print intrinsics (fx, fy, cx, cy).
@version 0.1
@date 2025-09-25
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

PROGRAM_MAJOR_VERSION = 1
PROGRAM_MINOR_VERSION = 0


class DepthLiteCameraInfoSubscriber(Node):
    """
    ROS2 node for subscribing to CameraInfo messages from Realsense camera.

    This node subscribes to the CameraInfo topic, stores the latest message,
    and prints camera intrinsic parameters (fx, fy, cx, cy) to verify callback functionality.
    """

    def __init__(self, node_name: str = "camera_info_subscriber"):
        """
        Initialize the DepthLiteCameraInfoSubscriber node.

        @param node_name (str): name of the ROS2 node
        """
        super().__init__(node_name)

        # topic name of the Realsense CameraInfo
        self._topic_name = "/realsense_camera_node/sn408122070053/camera_info"
        # use sensor data QoS profile
        qos = rclpy.qos.qos_profile_sensor_data

        self._subscription = self.create_subscription(
            CameraInfo, self._topic_name, self.info_callback, qos
        )
        # store the latest CameraInfo message
        self._camera_info_msg = None

        self.get_logger().info(
            f"CameraInfo subscriber started. topic='{self._topic_name}'"
        )

    def info_callback(self, msg: CameraInfo):
        """
        Callback function for CameraInfo topic.

        Extracts width, height, and intrinsic matrix parameters (fx, fy, cx, cy),
        then logs them for verification.

        @param msg (CameraInfo): ROS2 CameraInfo message
        """
        self._camera_info_msg = msg

        # extract intrinsics from the K matrix
        fx = msg.k[0] if len(msg.k) >= 6 else 0.0
        fy = msg.k[4] if len(msg.k) >= 6 else 0.0
        cx = msg.k[2] if len(msg.k) >= 6 else 0.0
        cy = msg.k[5] if len(msg.k) >= 6 else 0.0

        self.get_logger().info(
            f"CameraInfo received. width={msg.width} height={msg.height} "
            f"fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}"
        )


def main(args=None):
    """
    Main function.

    Initializes ROS2, creates DepthLiteCameraInfoSubscriber node, and spins it.
    Supports '-v' argument to print version only.
    """
    if len(sys.argv) > 1 and sys.argv[1] == "-v":
        version = f"{PROGRAM_MAJOR_VERSION}.{PROGRAM_MINOR_VERSION}"
        print(version)
        return

    rclpy.init(args=args)
    node = DepthLiteCameraInfoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
