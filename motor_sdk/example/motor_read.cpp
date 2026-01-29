/**
 * This file is Dobot Robotics's property. It contains Dobot Robotics trade secret, proprietary and confidential information.
 *
 * The information and code contained in this file is only for authorized Dobot Robotics employees to design, create, modify, or review.
 *
 * DO NOT DISTRIBUTE, DO NOT DUPLICATE OR TRANSMIT IN ANY FORM WITHOUT PROPER AUTHORIZATION.
 *
 * If you are not an intended recipient of this file, you must not copy, distribute, modify, or take any action in reliance on it.
 *
 * If you have received this file in error, please immediately notify Dobot Robotics and permanently delete the original and any copy of any file and any printout thereof.
 *
 * Copyright (c) 2025 Dobot Robotics
 *
 *
 * Author: Yixuan Chen
 * Mail: chenyixuan@dobot-robots.com
 * Created on Thu Sep 25 2025
 */

/**
 * @file examples/motor_read.cpp
 * @brief Minimal control loop that sets damping mode and prints joint positions/torques in place.
 * @details
 *  - Runs the SDK at 500 Hz (dt = 0.002) with a user-provided control callback.
 *  - Each cycle: apply damping command, clear console (ANSI) and print q/tau.
 *  - Robot type is selected by argv[1] (birdy1/birdy1L/miniHex/miniHex_v2).
 */

#include <vector>
#include <chrono>
#include <iomanip>
#include <stdint.h>
#include <iostream>
#include <thread>
#include <math.h>
#include <functional>
#include "robot_sdk.h"

class Custom
{
public:
    /**
     * @brief set the robot to damping mode for protection
     *
     * @param command
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-25
     * @copyright Copyright (c) 2025
     */
    void damp(RobotCommand &command)
    {
        command.qDes.setZero();
        command.qdDes.setZero();
        command.tauDes.setZero();
        command.kpDes.setZero();
        command.kdDes.setConstant(0.5);
    }
    /**
     * @brief Control callback invoked by RobotSDK at each cycle.
     *
     * @param state
     * @param command
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-25
     * @copyright Copyright (c) 2025
     * NOTE: Full screen clear (\033[2J) + home (\033[H]) is simple but can flicker on slow terminals.
     *        If desired, switch to a smaller in-place update using cursor-up sequences to reduce flicker.
     */
    void controlFunction(const RobotState &state, RobotCommand &command)
    {
        // Apply damping mode
        damp(command);

        // ANSI: clear entire screen and move cursor to top-left
        std::cout << "\033[2J\033[H";

        // Pretty printing with fixed precision
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "q: " << state.q.transpose() << std::endl;
        std::cout << "torque: " << state.tau.transpose() << std::endl;
    }
};

int main(int argc, char *argv[])
{

    std::string config_path;

    if (argc == 2)
    {

        if (0 == strcmp("-v", argv[1]))
        {
            RobotSDK robot;
            printf("robot sdk version: %s\n", robot.get_version().c_str());
            return 0;
        }
        else if (0 == strcmp("birdy1", argv[1]))
        {
            config_path = "/home/robot/robot_controller_release/executable/config/joint_config_birdy.yaml";
        }
        else if (0 == strcmp("birdy1L", argv[1]))
        {
            config_path = "/home/robot/robot_controller_release/executable/config/joint_config_birdy_lower_limb.yaml";
        }
        else if (0 == strcmp("miniHex", argv[1]))
        {
            config_path = "/home/robot/robot_controller_release/executable/config/joint_config_miniHex.yaml";
        }
        else if (0 == strcmp("miniHex_v2", argv[1]))
        {
            config_path = "/home/robot/robot_controller_release/executable/config/joint_config_mini_hex_v2.yaml";
        }
        else
        {
            std::cout << "Nonvalid robot type !!! Use birdy1/birdy1L/miniHex/miniHex_v2" << std::endl;
            exit(1);
        }
    }
    else
    {
        std::cout << "Select robot type!!!" << std::endl;
        exit(1);
    }

    Custom cust;
    RobotSDK robot(config_path, 0.002, std::bind(&Custom::controlFunction, &cust, std::placeholders::_1, std::placeholders::_2));

    robot.init();
    robot.start();

    while (1)
    {
        sleep(10);
    }
    //    robot.stop();
    return 0;
}