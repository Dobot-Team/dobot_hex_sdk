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
@file camera_fps.py
@author  Yixuan Chen
@brief Subscribe Realsense depth (mono16) and BGR image topics, calculate FPS, and print info.
       (NOTE: This Python port prints FPS like the C++ version; PGM saving stub left for extension.)
@version 0.1
@date 2025-09-26
"""
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


PROGRAM_MAJOR_VERSION = 1
PROGRAM_MINOR_VERSION = 0


class DepthLiteCameraFPSSubscriber(Node):
    """
    ROS2 node for subscribing to Realsense depth and BGR images.

    This node:
      - Subscribes to mono16 depth and BGR image topics
      - Computes instantaneous and smoothed FPS (EMA)
      - Logs FPS and image encoding/size periodically
    """

    def __init__(self, node_name: str = "camera_fps_subscriber"):
        """
        Initialize the DepthLiteCameraFPSSubscriber node.

        @param node_name (str): name of the ROS2 node
        """
        super().__init__(node_name)

        # params
        self.depth_topic = "/realsense_camera_node/sn408122070053/depth/u16/image_raw"
        self.bgr_topic = "/realsense_camera_node/sn408122070053/color/bgr/image_raw"

        # fps state
        now = self.get_clock().now()
        self._last_depth_stamp = now
        self._last_bgr_stamp = now
        self._ema_fps_depth: float = 0.0
        self._ema_fps_bgr: float = 0.0
        self._cnt: int = 0

        qos = rclpy.qos.qos_profile_sensor_data

        # subscribers
        self._sub_depth = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos
        )
        self._sub_bgr = self.create_subscription(
            Image, self.bgr_topic, self.bgr_callback, qos
        )

        self.get_logger().info(
            f"Subscribers started. depth='{self.depth_topic}', bgr='{self.bgr_topic}'"
        )

    @staticmethod
    def _secs_between(now, last) -> float:
        """
        Compute seconds between two ROS2 time stamps.

        @param now (rclpy.time.Time): current timestamp
        @param last (rclpy.time.Time): previous timestamp
        @return float: time difference in seconds
        """
        return (now - last).nanoseconds / 1e9

    @staticmethod
    def _ema(prev: float, x: float, alpha: float = 0.2) -> float:
        """
        Exponential moving average.

        @param prev (float): previous EMA value
        @param x (float): new input value
        @param alpha (float): smoothing factor
        @return float: updated EMA value
        """
        return x if prev == 0.0 else (alpha * x + (1.0 - alpha) * prev)

    def depth_callback(self, msg: Image):
        """
        Callback for depth image topic.

        Calculates instantaneous and smoothed FPS for depth stream,
        and logs results every other callback.

        @param msg (Image): ROS2 Image message
        """
        now = self.get_clock().now()
        dt = self._secs_between(now, self._last_depth_stamp)
        self._last_depth_stamp = now

        if dt > 1e-6:
            inst = 1.0 / dt
            self._ema_fps_depth = self._ema(self._ema_fps_depth, inst, 0.2)

        self._cnt += 1
        if self._cnt % 2 == 1:
            inst_show = (1.0 / dt) if dt > 1e-6 else 0.0
            self.get_logger().info(
                f"FPS: {self._ema_fps_depth:.2f} (inst={inst_show:.2f}) | "
                f"enc={msg.encoding} WxH={msg.width}x{msg.height}"
            )

    def bgr_callback(self, msg: Image):
        """
        Callback for BGR image topic.

        Calculates instantaneous and smoothed FPS for BGR stream,
        and logs results every other callback.

        @param msg (Image): ROS2 Image message
        """
        now = self.get_clock().now()
        dt = self._secs_between(now, self._last_bgr_stamp)
        self._last_bgr_stamp = now

        if dt > 1e-6:
            inst = 1.0 / dt
            self._ema_fps_bgr = self._ema(self._ema_fps_bgr, inst, 0.2)

        self._cnt += 1
        if self._cnt % 2 == 0:
            inst_show = (1.0 / dt) if dt > 1e-6 else 0.0
            self.get_logger().info(
                f"FPS: {self._ema_fps_bgr:.2f} (inst={inst_show:.2f}) | "
                f"enc={msg.encoding} WxH={msg.width}x{msg.height}"
            )


def main(args=None):
    # 版本输出
    if len(sys.argv) > 1 and sys.argv[1] == "-v":
        print(f"{PROGRAM_MAJOR_VERSION}.{PROGRAM_MINOR_VERSION}")
        return

    rclpy.init(args=args)
    node = DepthLiteCameraFPSSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
