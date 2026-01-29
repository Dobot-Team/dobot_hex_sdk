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
import sys
import time
import math

# -----------------------------------------------------------------------------
# Runtime path selection: onboard vs. offboard development
# -----------------------------------------------------------------------------
ONBOARD = True

if ONBOARD:
    sys.path.append(
        "/home/robot/robot_controller_release/third_party/motor_sdk/lib/python"
    )
else:
    sys.path.append(
        "/home/robot/robot_ws/biped_controller/modules/third_party/motor_sdk/lib/python"
    )

from robot_interface import RobotInterface


class controller:
    """
    Minimal joint-space motion demo:

    - Initializes RobotInterface with a selected YAML config.
    - Latches initial joint positions (q_init).
    - Runs a finite-duration sinusoidal position command around q_init.
    - Maintains a fixed control period `self._dt`.
    """

    def __init__(self) -> None:
        """Initialize paths, interface, buffers, and capture initial joint pose."""
        self._file_path = ""

        # Version query: `python script.py -v`
        if len(sys.argv) == 2:
            if sys.argv[1] == "-v":
                self._interface = RobotInterface()
                print("robot sdk version: " + self._interface.get_version())
                exit(1)

        # Config path based on environment
        if ONBOARD:
            self._file_path = "/home/robot/robot_controller_release/executable/config/joint_config_mini_hex_v2.yaml"
        else:
            self._file_path = "/home/robot/robot_ws/biped_controller/modules/config/joint_config_mini_hex_v2.yaml"

        # Control period (seconds)
        self._dt = 0.0021

        # Robot interface and hardware layout
        self._interface = RobotInterface(self._file_path)
        self._motor_num = self._interface.get_motor_num()

        # Command buffers (joint-space PD + feedforward torque)
        self._kpDes = np.zeros(self._motor_num)
        self._kdDes = np.zeros(self._motor_num)
        self._qDes = np.zeros(self._motor_num)
        self._qdDes = np.zeros(self._motor_num)
        self._tauDes = np.zeros(self._motor_num)

        # Observation buffers:
        # joints: motor_num * 3  (per motor: [q, qd, tau])
        # imu:    10             (SDK-specific)
        self._observation = np.zeros(self._motor_num * 3 + 10)
        self._joint_init = np.zeros(self._motor_num * 3)
        self._q_init = np.zeros(self._motor_num)

        # Timing
        self._last_step_time = time.time()

        # Capture initial joint positions over a short window
        self._count = 0
        for t in np.arange(0.0, 1.0, self._dt):
            # print("----------- init ", self._count, "---------- ")
            self._count += 1
            self._observation = self._interface.receive_observation()
            self._joint_init = self._observation[0 : self._motor_num * 3]
            # Reshape to (motor_num, 3) and take column 0 → q
            self._q_init = np.reshape(self._joint_init, (self._motor_num, 3))[:, 0]
            self.wait()

    def step(self):
        """
        Run the motion sequence:

        - Duration: 10 seconds
        - Trajectory: q_des = q_init + 0.2 * sin(2π s), where s = count / 500
        - Gains: kp = 5.0, kd = 0.25, qd_des = 0, tau_ff = 0
        """
        self._command = self._q_init
        self._count = 0
        for t in np.arange(0.0, 10.0, self._dt):
            # print("----------- step ", self._count, "---------- ")
            s = self._count / 500.0  # time-like scalar for the sine
            self._count += 1
            for idx in np.arange(0, self._motor_num, 1):
                self._kpDes[idx] = 5.0
                self._kdDes[idx] = 0.25
                self._qDes[idx] = self._q_init[idx] + math.sin(2 * math.pi * s) * 0.2
                self._qdDes[idx] = 0.0
                self._tauDes[idx] = 0.0

            # Send command → receive observation → hold rate
            self._interface.send_command(
                self._kpDes, self._kdDes, self._qDes, self._qdDes, self._tauDes
            )
            self._observation = self._interface.receive_observation()
            self.wait()

    def wait(self):
        """
        Sleep for the remainder of the control period `self._dt`.

        Prints a warning if the previous cycle exceeded the period
        (i.e., negative remaining time).
        """
        now = time.time()
        sleep_time = self._dt - (now - self._last_step_time)

        if sleep_time >= 0:
            time.sleep(sleep_time)
        else:
            print("[WARNNING]motor control overtime!")

        self._last_step_time = time.time()


if __name__ == "__main__":
    con = controller()
    con.step()
