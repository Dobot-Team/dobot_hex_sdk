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
 * 1. file function
 * @file body_twist/main.cpp
 * @author  Yixuan Chen  (chenyixuan@dobot-robots.com)
 * @brief for publishing joystick command to test the balanceStand state
 * @version 0.1
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2025
 *
 */

/**
 * 2.C/C++ basic header files declaration
 */
#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

/**
 * 3.ROS2 module headers declaration
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

/**
 * 4.Third party lib declaration
 */
#include <custom_msg/msg/robot_state.hpp>
#include <custom_msg/msg/robot_command.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

class bodyTwistFuncNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new body Twist Func Node object
     *
     * @param strNodeName
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-25
     * @copyright Copyright (c) 2025
     */
    bodyTwistFuncNode(const std::string &strNodeName) : Node(strNodeName)
    {
        using namespace std::literals::chrono_literals;
        using std::placeholders::_1;

        auto qos = rclcpp::QoS(1);

        try
        {
            // timer: call the control function frequently
            m_pPublishTopicTimer = this->create_wall_timer(20ms, std::bind(&bodyTwistFuncNode::bodyTwistControlMsgFunc, this));

            // publish robot_command topic
            m_pRobotCommandPublisher = this->create_publisher<custom_msg::msg::RobotCommand>(m_strRobotCommandTopicName, 1);

            // publish the joystick command
            m_pJoyTopicPublisher = this->create_publisher<sensor_msgs::msg::Joy>(m_strHandleTopicName, 1);

            // subscribe robot_state topic
            m_pRobotStateSubscriber = this->create_subscription<custom_msg::msg::RobotState>(m_strRobotStateTopicName, 1, std::bind(&bodyTwistFuncNode::parseRobotStateMsgFunc, this, _1));
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Fatal error during initialization: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Standard exception during IO initialization: %s", e.what());
            rclcpp::shutdown();
            return;
        }
    }

private:
    typedef enum RobotStateEnum
    {
        PASSIVE = 0,
        STAND_DOWN = 1,
        STAND_UP,
        BALANCE_STAND,
        WALK
    } T_RobotStateEnum;

    // topic name configuration
    std::string m_strRobotCommandTopicName = "/robot_cmd";
    std::string m_strHandleTopicName = "/joy";
    std::string m_strRobotStateTopicName = "/robot_state";

    // node info
    std::string m_strNodeName = "node_name_to_be_defined"; // default
    std::string m_strNodeVersion = "V" + std::to_string(PROGRAM_MAJOR_VERSION) + std::string(".") + std::to_string(PROGRAM_MINOR_VERSION);

    custom_msg::msg::RobotState m_msgRobotState; // save the robot state msg

    float count = 0.f; // for periodic signals

    // ROS2 communication handles
    rclcpp::TimerBase::SharedPtr m_pPublishTopicTimer = nullptr;
    rclcpp::Publisher<custom_msg::msg::RobotCommand>::SharedPtr m_pRobotCommandPublisher = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr m_pJoyTopicPublisher = nullptr;
    rclcpp::Subscription<custom_msg::msg::RobotState>::SharedPtr m_pRobotStateSubscriber = nullptr;

    void bodyTwistControlMsgFunc(void);
    void parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg);
};

/**
 * @brief update robotState to get the current mode
 *
 * @param msg
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-25
 * @copyright Copyright (c) 2025
 */
void bodyTwistFuncNode::parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg)
{
    m_msgRobotState.temp[10] = msg->temp[10]; // temp[10] represents the robot mode
    RCLCPP_INFO(this->get_logger(), "Subscribe Robot State Msg, m_msgRobotState.temp[10] = %f", m_msgRobotState.temp[10]);
}

/**
 * @brief function that controls the robot to twist its body
 *
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-25
 * @copyright Copyright (c) 2025
 */
void bodyTwistFuncNode::bodyTwistControlMsgFunc(void)
{
    // ensure robot is in balanceStand state
    if (m_msgRobotState.temp[10] < BALANCE_STAND)
    {
        custom_msg::msg::RobotCommand cmdMsg;
        cmdMsg.target_state = m_msgRobotState.temp[10] + 1;
        m_pRobotCommandPublisher->publish(cmdMsg);
        sleep(1); // sleep for robot state changing
    }
    else
    {
        count = (count <= 10000.) ? count + 1. : 0.;

        // swing frequency for each
        const float f_roll = 0.012;
        const float f_pitch = 0.012;
        const float f_yaw = 0.012; // Hz：yaw 频率（Lx）
        const float f_z = 0.012;

        sensor_msgs::msg::Joy joyMsg;
        joyMsg.header.stamp = this->now(); // header stamp
        joyMsg.axes.assign(8, 0.0f);
        joyMsg.buttons.assign(11, 0);
        joyMsg.axes[2] = 1.0f;
        joyMsg.axes[5] = 1.0f;

        // simulation
        joyMsg.axes[3] = std::clamp(static_cast<float>(0.7 * cos(2 * M_PI * f_roll * count)), -1.0f, 1.0f);  // roll
        joyMsg.axes[4] = std::clamp(static_cast<float>(0.7 * cos(2 * M_PI * f_z * count)), -1.0f, 1.0f);     // pitch
        joyMsg.axes[0] = std::clamp(static_cast<float>(0.7 * sin(2 * M_PI * f_yaw * count)), -1.0f, 1.0f);   // yaw
        joyMsg.axes[1] = std::clamp(static_cast<float>(0.7 * sin(2 * M_PI * f_pitch * count)), -1.0f, 1.0f); // z

        m_pJoyTopicPublisher->publish(joyMsg);
    }

    return;
}

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (std::string(argv[1]) == "-v")
        {
            std::string version = std::to_string(PROGRAM_MAJOR_VERSION) + std::string(".") + std::to_string(PROGRAM_MINOR_VERSION);
            std::cout << version << std::endl;
            return 0;
        }
    }

    rclcpp::init(argc, argv);

    // create node and start
    auto node = std::make_shared<bodyTwistFuncNode>("body_twist_control_example");
    RCLCPP_INFO(node->get_logger(), "Starting publisher node...");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}