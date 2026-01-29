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
@file lidar_fps_pc.py
@brief Subscribe a PointCloud2 and print FPS + point count (no publish, no extra deps).
@version 0.1
@date 2025-09-25
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time

PROGRAM_MAJOR_VERSION = 1
PROGRAM_MINOR_VERSION = 0


class PCLiteFPSCount(Node):
    """
    ROS2 node for monitoring PointCloud2 stream quality.

    This node:
      - Subscribes to a PointCloud2 topic
      - Computes instantaneous FPS and EMA-smoothed FPS
      - Verifies point count using both width*height and raw buffer size
      - Prints results at a throttled frequency (print_hz)
    """

    def __init__(self):
        """
        Initialize the node, create subscription and runtime state.
        """
        super().__init__("lidar_fps_count")

        # PointCloud2 topic to subscribe (adjust to your driver/topic naming)
        self.cloud_topic = "livox_Lidar_node/sn105/xyz/pointcloud"
        self.print_hz = 1.0  # 每秒打印一次

        # subscribe
        qos = rclpy.qos.qos_profile_sensor_data
        self.sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_callback, qos
        )

        self.last_stamp = self.get_clock().now()  # last msg timestamp
        self.ema_fps = 0.0  # EMA-smoothed FPS
        self.last_print_sec = 0.0  # last time we printed (sec)

        self.get_logger().info(
            f"pc_fps_count started. topic='{self.cloud_topic}', print_hz={self.print_hz:.2f}"
        )

    def cloud_callback(self, msg: PointCloud2):
        """
        PointCloud2 callback: compute FPS, estimate point count, and print.

        @param msg: PointCloud2 message.
                    - width, height: organized cloud dimensions
                    - point_step: bytes per point
                    - data: raw byte buffer
        """

        # FPS (instant + EMA)
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds / 1e9
        self.last_stamp = now

        if dt > 1e-6:
            inst_fps = 1.0 / dt
            alpha = 0.2  # EMA smoothing factor (0..1). Larger -> more responsive.
            self.ema_fps = (
                inst_fps
                if self.ema_fps == 0.0
                else alpha * inst_fps + (1 - alpha) * self.ema_fps
            )

        # Count by advertised width*height (works for organized clouds; height=1 for unorganized)
        count_wh = msg.width * msg.height

        # Count by raw buffer size (robust even if width*height is misleading)
        count_bytes = 0
        if msg.point_step > 0:
            count_bytes = len(msg.data) // msg.point_step

        now_s = now.nanoseconds / 1e9
        if now_s - self.last_print_sec >= 1.0 / max(0.1, self.print_hz):
            self.last_print_sec = now_s

            if count_wh != count_bytes and msg.point_step > 0:
                self.get_logger().warn(
                    f"[pc] FPS={self.ema_fps:.1f} | points(count_wh)={count_wh}, points(by_bytes)={count_bytes} "
                    f"(width={msg.width}, height={msg.height}, point_step={msg.point_step})"
                )
            else:
                self.get_logger().info(
                    f"[pc] FPS={self.ema_fps:.1f} | points={count_wh} (width={msg.width}, height={msg.height})"
                )


def main(args=None):
    if args and len(args) > 1 and args[1] == "-v":
        version = f"{PROGRAM_MAJOR_VERSION}.{PROGRAM_MINOR_VERSION}"
        print(version)
        return

    rclpy.init(args=args)
    node = PCLiteFPSCount()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
