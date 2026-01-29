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


def array_to_str_2f(arr: np.ndarray) -> str:
    """
    Format a 1D/2D array as a two-decimal string for compact console printing,
    aligning with C++ `std::fixed << setprecision(2)` style.

    Args:
        arr: 1D or 2D numeric array.

    Returns:
        str: e.g., "[ 0.00  1.25 -2.40]" or a multi-row representation.
    """
    arr = np.asarray(arr)
    return np.array2string(
        arr,
        formatter={"float_kind": lambda x: f"{x:.2f}"},
        separator=" ",
        max_line_width=200,
    )


class controller:
    """
    Minimal loop that:
    - Initializes RobotInterface with the selected config.
    - Sends damping commands (kp=0, kd=0.5, q/qd/tau=0).
    - Prints joint positions and torques in-place at a fixed control period.
    """

    def __init__(self) -> None:
        self._file_path = ""

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

        # Control period (s)
        self._dt = 0.0021

        # Robot interface and hardware layout
        self._interface = RobotInterface(self._file_path)
        self._motor_num = self._interface.get_motor_num()

        # Command buffers (damping mode)
        self._kpDes = np.zeros(self._motor_num, dtype=np.float32)
        self._kdDes = np.full(self._motor_num, 0.5, dtype=np.float32)
        self._qDes = np.zeros(self._motor_num, dtype=np.float32)
        self._qdDes = np.zeros(self._motor_num, dtype=np.float32)
        self._tauDes = np.zeros(self._motor_num, dtype=np.float32)

        # Timing anchor for fixed-rate loop
        self._last_step_time = time.time()

    def wait(self):
        """
        Sleep just enough to maintain the configured control period.
        Emits a warning if the previous cycle overran the period.
        """
        now = time.time()
        sleep_time = self._dt - (now - self._last_step_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print("[WARNNING] motor control overtime!")
        self._last_step_time = time.time()

    def step(self):
        """
        Blocking control loop:
        - Receive observation.
        - Parse joint block as [q, qd, tau] per motor.
        - Send damping command.
        - Clear screen and print q and tau with 2 decimals.
        - Rate-limit to _dt.
        """
        while True:
            # Receive observation vector
            obs = self._interface.receive_observation()

            # First motor_num*3 entries are joint block: [q, qd, tau] per motor
            joint_block = np.reshape(obs[0 : self._motor_num * 3], (self._motor_num, 3))
            q = joint_block[:, 0]
            tau = joint_block[:, 2]

            # Damping command (kp=0, kd=0.5; desired q/qd/tau = 0)
            self._interface.send_command(
                self._kpDes, self._kdDes, self._qDes, self._qdDes, self._tauDes
            )

            # Clear whole screen and move cursor to top-left (ANSI),
            # consistent with C++ "\033[2J\033[H"
            print("\033[2J\033[H", end="")

            # Two-decimal printing for compact readability
            print(f"q: {array_to_str_2f(q)}")
            print(f"torque: {array_to_str_2f(tau)}")

            # Maintain control rate
            self.wait()


if __name__ == "__main__":
    con = controller()
    con.step()
