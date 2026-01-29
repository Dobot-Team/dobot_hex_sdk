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
     * @brief
     *
     * @param state
     * @param command
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-25
     * @copyright Copyright (c) 2025
     */
    void controlFunction(const RobotState &state, RobotCommand &command)
    {
        if (control_iter < 10)
        {
            // Latch initial pose
            q_init = state.q;
            std::cout << "get q init " << q_init.transpose() << std::endl;
        }
        else if (control_iter < 5000)
        {
            // Time-like scalar for simple sinusoid
            double s = double(control_iter - 1000) / 500.0;

            // drive all joints with the same small-amplitude sinusoid
            for (int i = 0; i < q_init.size(); i++)
            {
                command.qDes(i) = q_init(i) + sin(2 * M_PI * s) * 0.2;
                command.qdDes.setZero();
                command.tauDes.setZero();
                command.kpDes.setConstant(5);
                command.kdDes.setConstant(0.25);
            }
        }
        else
        {
            // End of sequence
            damp(command);
            flag = true;
        }
        control_iter++;
    }
    /**
     * @brief Check whether the motion sequence has completed.
     * @return true if finished (controller switched to damping)
     */
    bool isFinished() { return flag; }

private:
    int control_iter = 0; // Counts control cycles
    DVecd q_init;         // Initial joint positions (type provided by SDK)
    bool flag = false;    // Completion flag
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

    // Controller instance and SDK wrapper with control callback
    Custom cust;
    RobotSDK robot(config_path, 0.002, std::bind(&Custom::controlFunction, &cust, std::placeholders::_1, std::placeholders::_2));

    // Initialize hardware and start control loop
    robot.init();
    robot.start();

    // Busy-wait until controller signals completion
    while (!cust.isFinished())
    {
    }

    robot.stop();
    return 0;
}
