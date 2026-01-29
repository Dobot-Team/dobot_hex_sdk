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
@file lidar_imu.py
@brief Subscribe Livox IMU, print FPS + processed key info (no publish, no extra deps).
@version 0.1
@date 2025-09-25
"""

import math
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

PROGRAM_MAJOR_VERSION = 1
PROGRAM_MINOR_VERSION = 0


class LivoxImuLite(Node):
    """
    ROS2 node that subscribes to a Livox IMU topic and prints processed IMU information.

    Features:
      - EMA-smoothed stream FPS
      - Stationary detection using ||a|-g| and |ω| thresholds
      - Online gyro bias estimation while stationary
      - Exponential low-pass filtering for accel/gyro
      - Roll/Pitch estimation from accelerometer (ignoring linear acceleration)
    """

    def __init__(self):
        """
        Initialize the IMU lite node, create the subscription, and set processing state.
        """
        super().__init__("livox_imu_lite")

        # topic and other params
        self.imu_topic = "/livox_Lidar_node/sn105/imu/raw_data"
        self.print_hz = 5.0  # console print rate (Hz)
        self.fps_alpha = 0.2  # EMA factor for FPS (0..1), larger -> more responsive
        self.filt_alpha = 0.2  # EMA factor for accel/gyro (0..1)

        # stationary detection & gyro-bias estimation thresholds
        self.g = 9.80665  # gravity (m/s^2)
        self.accel_tol = 0.25  # stationary if ||a|-g| < accel_tol  (m/s^2)
        self.gyro_tol = 0.03  # stationary if |ω| < gyro_tol       (rad/s) ~1.7 deg/s
        self.bias_samples_needed = 200  # samples needed for gyro bias

        # subscribe IMU
        qos = rclpy.qos.qos_profile_sensor_data
        self.sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos)

        self.last_stamp = self.get_clock().now()
        self.last_print_sec = 0.0
        self.fps_ema = 0.0

        self.ax_f = self.ay_f = self.az_f = 0.0
        self.gx_f = self.gy_f = self.gz_f = 0.0

        self.gbx = self.gby = self.gbz = 0.0  # gyro bias
        self.bias_count = 0

        self.get_logger().info(
            f"livox_imu_lite started. topic='{self.imu_topic}', print_hz={self.print_hz:.2f}"
        )

    @staticmethod
    def norm3(x: float, y: float, z: float) -> float:
        """
        Euclidean norm of a 3D vector.
        @return sqrt(x^2 + y^2 + z^2)
        """
        return math.sqrt(x * x + y * y + z * z)

    @staticmethod
    def rad2deg(r: float) -> float:
        """
        Convert radians to degrees.
        """
        return r * (180.0 / math.pi)

    def imu_callback(self, msg: Imu):
        """
        IMU callback: update FPS, detect stationary, estimate gyro bias, filter signals,
        compute roll/pitch, and print results at a throttled rate.

        @param msg: sensor_msgs/Imu
                    - linear_acceleration in m/s^2
                    - angular_velocity in rad/s
        """

        # FPS
        now = self.get_clock().now()
        dt = (now - self.last_stamp).nanoseconds / 1e9
        self.last_stamp = now
        if dt > 1e-6:
            fps_inst = 1.0 / dt
            self.fps_ema = (
                fps_inst
                if self.fps_ema == 0.0
                else (self.fps_alpha * fps_inst + (1.0 - self.fps_alpha) * self.fps_ema)
            )

        # raw data
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)

        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)

        # stationary detection (for bias estimation)
        a_mag = self.norm3(ax, ay, az)
        w_mag = self.norm3(gx, gy, gz)
        is_stationary = (abs(a_mag - self.g) < self.accel_tol) and (
            w_mag < self.gyro_tol
        )

        # Gyro bias estimation while stationary
        if is_stationary and self.bias_count < self.bias_samples_needed:
            k = 1.0 / float(self.bias_count + 1)
            self.gbx = (1.0 - k) * self.gbx + k * gx
            self.gby = (1.0 - k) * self.gby + k * gy
            self.gbz = (1.0 - k) * self.gbz + k * gz
            self.bias_count += 1

        # bias removal + exponential filtering
        gx_nb = gx - self.gbx
        gy_nb = gy - self.gby
        gz_nb = gz - self.gbz

        self.ax_f = self.filt_alpha * ax + (1.0 - self.filt_alpha) * self.ax_f
        self.ay_f = self.filt_alpha * ay + (1.0 - self.filt_alpha) * self.ay_f
        self.az_f = self.filt_alpha * az + (1.0 - self.filt_alpha) * self.az_f

        self.gx_f = self.filt_alpha * gx_nb + (1.0 - self.filt_alpha) * self.gx_f
        self.gy_f = self.filt_alpha * gy_nb + (1.0 - self.filt_alpha) * self.gy_f
        self.gz_f = self.filt_alpha * gz_nb + (1.0 - self.filt_alpha) * self.gz_f

        # Convention example (depends on IMU mounting): right-handed, x-forward, y-left, z-up
        # roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2))
        roll_rad = math.atan2(self.ay_f, self.az_f)
        pitch_rad = math.atan2(
            -self.ax_f, math.sqrt(self.ay_f * self.ay_f + self.az_f * self.az_f)
        )

        now_s = now.nanoseconds / 1e9
        if now_s - self.last_print_sec >= 1.0 / max(0.1, self.print_hz):
            self.last_print_sec = now_s

            a_err = abs(self.norm3(self.ax_f, self.ay_f, self.az_f) - self.g)
            w_deg = self.rad2deg(self.norm3(self.gx_f, self.gy_f, self.gz_f))

            self.get_logger().info(
                "[imu] FPS={:.1f} | a(m/s^2)=({:+.2f},{:+.2f},{:+.2f}) | |a|-g={:.2f} | "
                "w(deg/s)=({:+.2f},{:+.2f},{:+.2f}) | |w|={:.2f} | roll/pitch(deg)=({:.1f},{:.1f}) | "
                "stationary={} bias({}/{})".format(
                    self.fps_ema,
                    self.ax_f,
                    self.ay_f,
                    self.az_f,
                    a_err,
                    self.rad2deg(self.gx_f),
                    self.rad2deg(self.gy_f),
                    self.rad2deg(self.gz_f),
                    w_deg,
                    self.rad2deg(roll_rad),
                    self.rad2deg(pitch_rad),
                    "Y" if is_stationary else "N",
                    self.bias_count,
                    self.bias_samples_needed,
                )
            )


def main(args=None):
    if args is None:
        args = sys.argv
    if len(args) > 1 and args[1] == "-v":
        print(f"{PROGRAM_MAJOR_VERSION}.{PROGRAM_MINOR_VERSION}")
        return

    rclpy.init(args=args)
    node = LivoxImuLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
