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
 * @file lidar_fps_pc.cpp
 * @author  Yixuan Chen
 * @brief Subscribe a PointCloud2 topic and print FPS + point count.
 *        Provides EMA-smoothed FPS and verifies consistency between width*height and byte size.
 * @version 0.1
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2025
 *
 */

/**
 * 2. C/C++ basic headers
 */
#include <memory>
#include <string>

/**
 * 3. ROS2 headers
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

/**
 * @class PCLiteFPSCount
 * @brief ROS2 node for monitoring PointCloud2 FPS and point counts.
 *
 * This class subscribes to a Lidar PointCloud2 topic, computes instantaneous
 * and EMA-smoothed FPS, and prints point count statistics (by width*height
 * and by data size).
 */
class PCLiteFPSCount : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new PCLiteFPSCount node
     *
     * Initializes subscriber, state variables, and logs startup.
     * Default print frequency is once per second.
     */
    PCLiteFPSCount() : rclcpp::Node("lidar_fps_pc")
    {
        using std::placeholders::_1;

        m_printHz = 1.0; // print once per second

        auto qos = rclcpp::SensorDataQoS();
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_strPointCloudTopicName, qos, std::bind(&PCLiteFPSCount::cloudCallback, this, _1));

        last_stamp_ = this->now();
        ema_fps_ = 0.0;
        last_print_sec_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "pc_fps_count started. topic='%s', print_hz=%.2f",
                    m_strPointCloudTopicName.c_str(), m_printHz);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

private:
    // PointCloud2 topic name
    std::string m_strPointCloudTopicName = "/livox_Lidar_node/sn105/xyz/pointcloud";
    // Log print frequency (Hz)
    double m_printHz;

    // ROS2 subscription handle
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    // Last message timestamp
    rclcpp::Time last_stamp_;
    // EMA-smoothed FPS
    double ema_fps_;
    // Last log print time
    double last_print_sec_;
};

/**
 * @brief Callback for PointCloud2 messages.
 *
 * @param msg
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-26
 * @copyright Copyright (c) 2025
 */
void PCLiteFPSCount::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    const auto now = this->now();
    const double dt = (now - last_stamp_).seconds();
    last_stamp_ = now;

    // FPS calculation
    if (dt > 1e-6)
    {
        const double inst_fps = 1.0 / dt;
        const double alpha = 0.2; // EMA smoothing factor
        ema_fps_ = (ema_fps_ == 0.0) ? inst_fps : (alpha * inst_fps + (1.0 - alpha) * ema_fps_);
    }

    // point counts
    const std::size_t count_wh = static_cast<std::size_t>(msg->width) * msg->height;

    std::size_t count_bytes = 0;
    if (msg->point_step > 0)
    {
        count_bytes = msg->data.size() / msg->point_step;
    }

    // controlled print frequency
    const double now_s = now.seconds();
    if (now_s - last_print_sec_ >= 1.0 / std::max(0.1, m_printHz))
    {
        last_print_sec_ = now_s;

        if (count_wh != count_bytes && msg->point_step > 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "[pc] FPS=%.1f | points(count_wh)=%zu, points(by_bytes)=%zu (width=%u, height=%u, point_step=%u)",
                        ema_fps_, count_wh, count_bytes, msg->width, msg->height, msg->point_step);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "[pc] FPS=%.1f | points=%zu (width=%u, height=%u)",
                        ema_fps_, count_wh, msg->width, msg->height);
        }
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
    rclcpp::spin(std::make_shared<PCLiteFPSCount>());
    rclcpp::shutdown();
    return 0;
}