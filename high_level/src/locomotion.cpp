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
 * @file locomotion/main.cpp
 * @author  Yixuan Chen
 * @brief publish velocity command to test the robot locomotion function
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
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * 4.Third party lib declaration
 */
#include <custom_msg/msg/robot_state.hpp>
#include <custom_msg/msg/robot_command.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

class locomotionFuncNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new locomotion Func Node object
     *
     * @param strNodeName
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-25
     * @copyright Copyright (c) 2025
     */
    locomotionFuncNode(const std::string &strNodeName) : Node(strNodeName)
    {
        using namespace std::literals::chrono_literals;
        using std::placeholders::_1;

        auto qos = rclcpp::QoS(1);

        try
        {
            // Periodic timer callback (20ms, ~50Hz)
            m_pPublishTopicTimer = this->create_wall_timer(20ms, std::bind(&locomotionFuncNode::locomotionContrlMsgFunc, this));

            // Publisher for robot command messages
            m_pRobotCommandPublisher = this->create_publisher<custom_msg::msg::RobotCommand>(m_strRobotCommandTopicName, 1);

            // Publisher for velocity control (geometry_msgs::Twist)
            m_pRobotControllVelCmdPublisher = this->create_publisher<geometry_msgs::msg::Twist>(m_strVelCmdTopicName, 1);

            // Subscriber for robot state feedback
            m_pRobotStateSubscriber = this->create_subscription<custom_msg::msg::RobotState>(m_strRobotStateTopicName, 1, std::bind(&locomotionFuncNode::parseRobotStateMsgFunc, this, _1));
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
    /** @enum T_RobotStateEnum
     *  @brief Robot state definitions
     */
    typedef enum RobotStateEnum
    {
        PASSIVE = 0,    ///< Passive mode
        STAND_DOWN = 1, ///< Stand down
        STAND_UP,       ///< Stand up
        BALANCE_STAND,  ///< Balance standing
        WALK            ///< Walking
    } T_RobotStateEnum;

    // Topic names and node info
    std::string m_strRobotCommandTopicName = "/robot_cmd";
    std::string m_strRobotStateTopicName = "/robot_state";
    std::string m_strVelCmdTopicName = "/vel_cmd";
    std::string m_strNodeName = "node_name_to_be_defined"; // default
    std::string m_strNodeVersion = "V" + std::to_string(PROGRAM_MAJOR_VERSION) + std::string(".") + std::to_string(PROGRAM_MINOR_VERSION);

    // ROS2 communication handles
    rclcpp::TimerBase::SharedPtr m_pPublishTopicTimer = nullptr;
    rclcpp::Publisher<custom_msg::msg::RobotCommand>::SharedPtr m_pRobotCommandPublisher = nullptr;
    rclcpp::Subscription<custom_msg::msg::RobotState>::SharedPtr m_pRobotStateSubscriber = nullptr;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pRobotControllVelCmdPublisher = nullptr;

    bool first_run = true;

    custom_msg::msg::RobotState m_msgRobotState;
    // geometry_msgs::msg::Twist m_msgRobotControl;

    void locomotionContrlMsgFunc(void);
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
void locomotionFuncNode::parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg)
{
    m_msgRobotState.temp[10] = msg->temp[10]; // Robot State Get Byte
    RCLCPP_INFO(this->get_logger(), "Subscribe Robot State Msg, m_msgRobotState.temp[10] = %f", m_msgRobotState.temp[10]);
}

/**
 * @brief publish velocities of each axis
 *
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-25
 * @copyright Copyright (c) 2025
 */
void locomotionFuncNode::locomotionContrlMsgFunc(void)
{
    if (first_run && m_msgRobotState.temp[10] < WALK) // Robot State Check Byte
    {
        // Step robot state until WALK mode
        custom_msg::msg::RobotCommand cmdMsg;
        cmdMsg.target_state = m_msgRobotState.temp[10] + 1;
        m_pRobotCommandPublisher->publish(cmdMsg);
        sleep(1); // sleep for robot state changing
    }
    else
    {
        first_run = false;
        static rclcpp::Time t0 = this->now();
        static const double T = 2.0; // Duration of each motion segment (s)
        double dt = (this->now() - t0).seconds();

        // Motion profiles
        const float v_forward = +0.4f;
        const float v_backward = -0.4f;
        const float v_left = +0.4f;
        const float v_right = -0.4f;
        const float v_left_yaw = +0.8f;
        const float v_right_yaw = -0.8f;

        float fXSpeed = 0.0;
        float fYSpeed = 0.0;
        float fYawSpeed = 0.0;

        // Motion sequence state machine
        if (dt < T)
        {
            fXSpeed = v_forward;
        }
        else if (dt < 2 * T)
        {
            fXSpeed = v_backward;
        }
        else if (dt < 3 * T)
        {
            fYSpeed = v_left;
        }
        else if (dt < 4 * T)
        {
            fYSpeed = v_right;
        }
        else if (dt < 5 * T)
        {
            fYawSpeed = v_left_yaw;
        }
        else if (dt < 6 * T)
        {
            fYawSpeed = v_right_yaw;
        }
        else
        {
            if (m_msgRobotState.temp[10] > PASSIVE)
            {
                custom_msg::msg::RobotCommand cmdMsg;
                if (m_msgRobotState.temp[10] == BALANCE_STAND)
                {
                    cmdMsg.target_state = m_msgRobotState.temp[10] - 2;
                }
                else
                {
                    cmdMsg.target_state = m_msgRobotState.temp[10] - 1;
                }

                m_pRobotCommandPublisher->publish(cmdMsg);
                sleep(1); // sleep for robot state changing
            }
            return;
        }

        // Create and publish Twist message
        auto controlMsg = geometry_msgs::msg::Twist();
        // modify speed

        // controlMsg.linear.x = m_msgRobotControl.linear.x * 0.5 + fXSpeed * 0.5;
        // controlMsg.linear.y = m_msgRobotControl.linear.y * 0.6 + fYSpeed * 0.4;
        // controlMsg.angular.z = m_msgRobotControl.angular.z * 0.65 + fYawSpeed * 0.35;

        controlMsg.linear.x = fXSpeed * 0.5;
        controlMsg.linear.y = fYSpeed * 0.4;
        controlMsg.angular.z = fYawSpeed * 0.35;

        m_pRobotControllVelCmdPublisher->publish(controlMsg);
    }
}

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (std::string(argv[1]) == "-v")
        {
            std::string version = std::to_string(PROGRAM_MAJOR_VERSION) +
                                  std::string(".") +
                                  std::to_string(PROGRAM_MINOR_VERSION);
            std::cout << version << std::endl;
            return 0;
        }
    }

    rclcpp::init(argc, argv);

    auto node = std::make_shared<locomotionFuncNode>("locomotion_control_publisher");
    RCLCPP_INFO(node->get_logger(), "Starting publisher node...");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}