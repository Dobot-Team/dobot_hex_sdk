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
 * Created on Fri Sep 26 2025
 */

/**
 * 1. file function
 * @file camera_info.cpp
 * @author  Yixuan Chen
 * @brief Subscribe Realsense depth CameraInfo topic and print intrinsics.
 * @version 0.1
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2025
 *
 */

/**
 * 2.C/C++ basic header files declaration
 */
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

/**
 * 3.ROS2 module headers declaration
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

/**
 * @class DepthLiteCameraInfoSubscriber
 * @brief ROS2 node subscribing to CameraInfo and printing intrinsic parameters.
 */
class DepthLiteCameraInfoSubscriber : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Depth Lite Camera Info Subscriber object
     *
     * @param strNodeName
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-26
     * @copyright Copyright (c) 2025
     */
    DepthLiteCameraInfoSubscriber(const std::string &strNodeName) : Node(strNodeName)
    {
        using namespace std::literals::chrono_literals;
        using std::placeholders::_1;

        auto qos = rclcpp::SensorDataQoS();
        m_pCameraInfoSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(m_strDepthCameraInfoTopicName, qos, std::bind(&DepthLiteCameraInfoSubscriber::infoCallback, this, _1));
        RCLCPP_INFO(this->get_logger(),
                    "CameraInfo subscriber started. topic='%s'", m_strDepthCameraInfoTopicName.c_str());
    }

private:
    // CameraInfo topic name
    std::string m_strDepthCameraInfoTopicName = "/realsense_camera_node/sn408122070053/camera_info";

    // Latest CameraInfo message
    sensor_msgs::msg::CameraInfo m_msgCameraInfo;

    // Subscriber handle
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_pCameraInfoSubscriber = nullptr;
    void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info);
};

/**
 * @brief Callback function to process CameraInfo messages
 *
 * @param info
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-26
 * @copyright Copyright (c) 2025
 */
void DepthLiteCameraInfoSubscriber::infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info)
{
    m_msgCameraInfo = *info;
    // print CameraInfo intrinsics
    RCLCPP_INFO(this->get_logger(),
                "CameraInfo received. width=%u height=%u fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                m_msgCameraInfo.width,
                m_msgCameraInfo.height,
                (m_msgCameraInfo.k.size() >= 6 ? m_msgCameraInfo.k[0] : 0.0), // fx
                (m_msgCameraInfo.k.size() >= 6 ? m_msgCameraInfo.k[4] : 0.0), // fy
                (m_msgCameraInfo.k.size() >= 6 ? m_msgCameraInfo.k[2] : 0.0), // cx
                (m_msgCameraInfo.k.size() >= 6 ? m_msgCameraInfo.k[5] : 0.0)  // cy
    );
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::string(argv[1]) == "-v")
    {
        std::string version = std::to_string(PROGRAM_MAJOR_VERSION) + "." + std::to_string(PROGRAM_MINOR_VERSION);
        std::cout << version << std::endl;
        return 0;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthLiteCameraInfoSubscriber>("camera_info_subscriber"));
    rclcpp::shutdown();
    return 0;
}
