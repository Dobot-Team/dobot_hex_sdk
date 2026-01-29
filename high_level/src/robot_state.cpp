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
 * @file robot_state/main.cpp
 * @author  Yixuan Chen
 * @brief pring in console and update the robot state in place
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
#include <sstream>
#include <iomanip>
#include <array>
#include <vector>

/**
 * 3.ROS2 module headers declaration
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * 4.Third party lib declaration
 */
#include <custom_msg/msg/robot_state.hpp>
#include <custom_msg/msg/robot_command.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

/**
 * @brief Convert std::array to string for table printing
 * @tparam T element type
 * @tparam N array size
 * @param a array reference
 * @param prec decimal precision
 * @return std::string formatted string
 */
template <typename T, std::size_t N>
static std::string arr_to_str(const std::array<T, N> &a, int prec = 3)
{
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(prec) << "[";
    for (std::size_t i = 0; i < N; ++i)
    {
        if (i)
            oss << ", ";
        oss << a[i];
    }
    oss << "]";
    return oss.str();
}

/**
 * @class robotStateFuncNode
 * @brief ROS2 node for robot state monitoring
 *
 * Responsibilities:
 * - Subscribe to /robot_state
 * - Print formatted state values in-place in console
 * - Provide a publisher for /robot_cmd (not used actively here)
 */
class robotStateFuncNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new robot State Func Node object
     *
     * @param strNodeName
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-25
     * @copyright Copyright (c) 2025
     */
    robotStateFuncNode(const std::string &strNodeName) : Node(strNodeName)
    {
        using namespace std::literals::chrono_literals;
        using std::placeholders::_1;

        auto qos = rclcpp::QoS(1);

        try
        {
            // Timer: not actively used, only placeholder
            m_pPublishTopicTimer = this->create_wall_timer(20ms, std::bind(&robotStateFuncNode::robotCommandContrlMsgFunc, this));

            // Publisher for RobotCommand
            m_pRobotCommandPublisher = this->create_publisher<custom_msg::msg::RobotCommand>(m_strRobotCommandTopicName, 1);

            // Subscriber for RobotState
            m_pRobotStateSubscriber = this->create_subscription<custom_msg::msg::RobotState>(m_strRobotStateTopicName, 1, std::bind(&robotStateFuncNode::parseRobotStateMsgFunc, this, _1));
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
     *  @brief Robot state machine definition
     */
    typedef enum RobotStateEnum
    {
        PASSIVE = 0,
        STAND_DOWN = 1,
        STAND_UP,
        BALANCE_STAND,
        WALK
    } T_RobotStateEnum;

    // Topic names
    std::string m_strRobotCommandTopicName = "/robot_cmd";
    std::string m_strRobotStateTopicName = "/robot_state";

    // Node info
    std::string m_strNodeName = "node_name_to_be_defined"; // default
    std::string m_strNodeVersion = "V" + std::to_string(PROGRAM_MAJOR_VERSION) + std::string(".") + std::to_string(PROGRAM_MINOR_VERSION);

    // ROS2 handles
    rclcpp::TimerBase::SharedPtr m_pPublishTopicTimer = nullptr;
    rclcpp::Publisher<custom_msg::msg::RobotCommand>::SharedPtr m_pRobotCommandPublisher = nullptr;
    rclcpp::Subscription<custom_msg::msg::RobotState>::SharedPtr m_pRobotStateSubscriber = nullptr;

    custom_msg::msg::RobotState m_msgRobotState;

    void robotCommandContrlMsgFunc(void);
    void parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg);
};

/**
 * @brief update robotState and pring them in the console
 *
 * @param msg
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-25
 * @copyright Copyright (c) 2025
 */
void robotStateFuncNode::parseRobotStateMsgFunc(const custom_msg::msg::RobotState::SharedPtr msg)
{
    m_msgRobotState = *msg;

    m_msgRobotState.temp[10] = msg->temp[10]; // Robot State Get Byte

    /* ------------ Print to console and update in place ------------ */
    static rclcpp::Time last = this->now();
    if ((this->now() - last).seconds() < 0.5)
    {
        return; // throttle print to every 0.5s
    }
    last = this->now();

    // Table formatting
    const int w_name = 13;
    const int w_value = 36;

    auto print_row = [&](const std::string &name, const std::string &val)
    {
        std::cout << "| " << std::left << std::setw(w_name) << name
                  << " | " << std::left << std::setw(w_value) << val
                  << "|\n";
    };

    static bool printed_once = false;
    constexpr int TABLE_LINES = 9;

    if (printed_once)
    {
        // Move cursor up to overwrite table
        std::cout << "\x1b[" << TABLE_LINES << "A\r";
    }
    else
    {
        printed_once = true;
        std::cout << "control with gamepad and observe the states\n";
    }

    // Table header
    std::cout << "+" << std::string(w_name + 2, '-') << "+"
              << std::string(w_value + 1, '-') << "+\n";
    std::cout << "| " << std::left << std::setw(w_name) << "Name"
              << " | " << std::left << std::setw(w_value) << "Value"
              << "|\n";
    std::cout << "+" << std::string(w_name + 2, '-') << "+"
              << std::string(w_value + 1, '-') << "+\n";

    // State values
    print_row("pos_body", arr_to_str(msg->pos_body));
    print_row("vel_body", arr_to_str(msg->vel_body));
    print_row("acc_body", arr_to_str(msg->acc_body));
    print_row("ori_body", arr_to_str(msg->ori_body)); // 四元数 [x,y,z,w]
    print_row("omega_body", arr_to_str(msg->omega_body));

    // Table footer
    std::cout << "+" << std::string(w_name + 2, '-') << "+"
              << std::string(w_value + 1, '-') << "+\n";

    std::cout << std::flush;
}

/**
 * @brief Timer callback (currently does nothing). Users need to control the robot with gamepad.
 *
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-25
 * @copyright Copyright (c) 2025
 */
void robotStateFuncNode::robotCommandContrlMsgFunc(void)
{
    // use controller to control and observe the robot states
    return;
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

    auto node = std::make_shared<robotStateFuncNode>("robot_state_subscriber");
    RCLCPP_INFO(node->get_logger(), "Main robot states as below");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}