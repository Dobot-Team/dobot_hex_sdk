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
 * @file camera_fps.cpp
 * @author  Yixuan Chen  (chenyixuan@dobot-robots.com)
 * @brief Subscribe Realsense depth and BGR images, calculate FPS and print basic info.
 * @version 0.1
 * @date 2025-09-26
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
#include <fstream>
#include <filesystem>
#include <iostream>

/**
 * 3.ROS2 module headers declaration
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

/**
 * @class DepthLiteCameraFPSSubscriber
 * @brief ROS2 node for subscribing depth and BGR image topics, calculating FPS and printing info.
 *
 * This class subscribes to Realsense depth (mono16) and BGR image topics.
 * It computes instantaneous FPS and smoothed FPS (EMA) and prints them periodically.
 */
class DepthLiteCameraFPSSubscriber : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Depth Lite Subscriber object
     *
     * @param strNodeName
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-26
     * @copyright Copyright (c) 2025
     */
    DepthLiteCameraFPSSubscriber(const std::string &strNodeName) : Node(strNodeName)
    {
        using std::placeholders::_1;

        // subscriber
        auto qos = rclcpp::SensorDataQoS();
        m_pDepthImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
            m_strDepthImageTopicName, qos, std::bind(&DepthLiteCameraFPSSubscriber::depthCallback, this, _1));
        m_pBGRImageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
            m_strBGRImageTopicName, qos, std::bind(&DepthLiteCameraFPSSubscriber::bgrCallback, this, _1));

        // fps init
        m_lastStamp_depth = this->now();
        m_lastStamp_bgr = this->now();
        m_emaFps_depth = 0.0;
        m_emaFps_bgr = 0.0;
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void bgrCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

private:
    // topic names
    std::string m_strDepthImageTopicName = "/realsense_camera_node/sn408122070053/depth/u16/image_raw";
    std::string m_strBGRImageTopicName = "/realsense_camera_node/sn408122070053/color/bgr/image_raw";

    // subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_pDepthImageSubscriber = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_pBGRImageSubscriber = nullptr;

    // fps
    rclcpp::Time m_lastStamp_depth;
    rclcpp::Time m_lastStamp_bgr;
    double m_emaFps_depth;
    double m_emaFps_bgr;

    int cnt = 0; // counter for alternating log outputs
};

/**
 * @brief Depth image callback, calculate FPS and print info
 *
 * @param msg
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-26
 * @copyright Copyright (c) 2025
 */
void DepthLiteCameraFPSSubscriber::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    // FPS calculation
    const auto now = this->now();
    const double dt = (now - m_lastStamp_depth).seconds();
    m_lastStamp_depth = now;

    if (dt > 1e-6)
    {
        const double inst_fps = 1.0 / dt;
        const double alpha = 0.2; // EMA 平滑
        m_emaFps_depth = (m_emaFps_depth == 0.0) ? inst_fps
                                                 : (alpha * inst_fps + (1.0 - alpha) * m_emaFps_depth);
    }
    // log information
    cnt += 1;
    if (cnt % 2 == 1)
    {
        RCLCPP_INFO(this->get_logger(),
                    "FPS: %.2f (inst=%.2f) | enc=%s WxH=%ux%u",
                    m_emaFps_depth,
                    (dt > 1e-6 ? 1.0 / dt : 0.0),
                    msg->encoding.c_str(), msg->width, msg->height);
    }
}

/**
 * @brief BGR image callback, calculate FPS and print info
 *
 * @param msg
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-26
 * @copyright Copyright (c) 2025
 */
void DepthLiteCameraFPSSubscriber::bgrCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    // FPS calculation
    const auto now = this->now();
    const double dt = (now - m_lastStamp_bgr).seconds();
    m_lastStamp_bgr = now;

    if (dt > 1e-6)
    {
        const double inst_fps = 1.0 / dt;
        const double alpha = 0.2; // EMA smooth factor
        m_emaFps_bgr = (m_emaFps_bgr == 0.0) ? inst_fps
                                             : (alpha * inst_fps + (1.0 - alpha) * m_emaFps_bgr);
    }
    // log information
    cnt += 1;
    if (cnt % 2 == 0)
    {
        RCLCPP_INFO(this->get_logger(),
                    "FPS: %.2f (inst=%.2f) | enc=%s WxH=%ux%u",
                    m_emaFps_bgr,
                    (dt > 1e-6 ? 1.0 / dt : 0.0),
                    msg->encoding.c_str(), msg->width, msg->height);
    }
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
    rclcpp::spin(std::make_shared<DepthLiteCameraFPSSubscriber>("camera_fps_subscriber"));
    rclcpp::shutdown();
    return 0;
}
