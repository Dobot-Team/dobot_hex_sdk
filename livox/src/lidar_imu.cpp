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
 * Author: Yixuan Chen
 * Mail: chenyixuan@dobot-robots.com
 * Created on Thu Sep 25 2025
 */

/**
 * 1. file function
 * @file livox_imu_lite.cpp
 * @author  Yixuan Chen
 * @brief Subscribe Livox IMU, print FPS + processed key info (no publish, no extra deps).
 *        Provides EMA-smoothed FPS, stationary detection, gyro bias estimation,
 *        low-pass filtering and roll/pitch estimation from accelerometer.
 * @version 0.1
 * @date 2025-09-25
 */

/**
 * 2. C/C++ basic headers
 */

#include <cmath>
#include <cstdint>
#include <string>
#include <memory>
#include <algorithm>
/**
 * 3. ROS2 headers
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#define PROGRAM_MAJOR_VERSION 1
#define PROGRAM_MINOR_VERSION 0

/**
 * @class LivoxImuLite
 * @brief ROS2 node that subscribes to a Livox IMU topic and prints processed IMU info.
 *
 * Features:
 * - EMA-smoothed stream FPS
 * - Stationary detection using |a|-g and |w| thresholds
 * - Online gyro bias estimation during stationary periods
 * - Exponential low-pass filtering for accel/gyro
 * - Roll/Pitch estimation from accelerometer (ignoring linear acceleration)
 */
class LivoxImuLite : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Livox Imu Lite object
     *
     * @version 0.1
     * @author  Yixuan Chen
     * @date 2025-09-26
     * @copyright Copyright (c) 2025
     */
    LivoxImuLite() : rclcpp::Node("livox_imu_lite")
    {
        using std::placeholders::_1;

        print_hz_ = 5.0;   // Console print frequency (Hz)
        fps_alpha_ = 0.2;  // EMA smoothing factor for FPS (0~1)
        filt_alpha_ = 0.2; // EMA smoothing factor for accel/gyro (0~1)

        // stationary detection & gyro bias estimation
        g_ = 9.80665;               // brief Gravity constant (m/s^2)
        accel_tol_ = 0.25;          // Stationary if | |a|-g | < accel_tol_ (m/s^2)
        gyro_tol_ = 0.03;           // Stationary if |w| < gyro_tol_ (rad/s)
        bias_samples_needed_ = 200; // Samples needed for gyro bias estimation

        // subscriber
        auto qos = rclcpp::SensorDataQoS();
        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, std::bind(&LivoxImuLite::imuCallback, this, _1));

        // runtime states
        last_stamp_ = this->now();
        last_print_sec_ = 0.0;
        fps_ema_ = 0.0;

        ax_f_ = ay_f_ = az_f_ = 0.0;
        gx_f_ = gy_f_ = gz_f_ = 0.0;

        gbx_ = gby_ = gbz_ = 0.0; // gyro bias
        bias_count_ = 0;

        RCLCPP_INFO(this->get_logger(),
                    "livox_imu_lite started. topic='%s', print_hz=%.2f", imu_topic_.c_str(), print_hz_);
    }

private:
    // ------------------------ utilities ------------------------

    /**
     * @brief Euclidean norm of 3D vector.
     * @param x X
     * @param y Y
     * @param z Z
     * @return double sqrt(x^2 + y^2 + z^2)
     */
    static inline double norm3(double x, double y, double z)
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    /**
     * @brief Radians to degrees.
     * @param r angle in radians
     * @return double angle in degrees
     */
    static inline double rad2deg(double r)
    {
        return r * (180.0 / M_PI);
    }
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

private:
    // IMU topic name
    std::string imu_topic_ = "/livox_Lidar_node/sn105/imu/raw_data";
    double print_hz_;
    // EMA smoothing factors
    double fps_alpha_;
    double filt_alpha_;

    double g_;
    double accel_tol_;
    double gyro_tol_;
    int bias_samples_needed_;

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

    rclcpp::Time last_stamp_;
    double fps_ema_;
    double last_print_sec_; // Last print time in seconds

    double ax_f_, ay_f_, az_f_; // filtered accelerations (m/s^2)
    double gx_f_, gy_f_, gz_f_; // filtered angular rates (rad/s)
    double gbx_, gby_, gbz_;    // estimated gyro biases (rad/s)
    int bias_count_;
};

/**
 * @brief IMU message callback: compute FPS, stationary detection, bias estimation,
 *        filtering, and roll/pitch; then print aggregated info at print_hz_.
 *
 * @param msg
 * @version 0.1
 * @author  Yixuan Chen
 * @date 2025-09-26
 * @copyright Copyright (c) 2025
 */
void LivoxImuLite::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    // FPS update
    const auto now = this->now();
    const double dt = (now - last_stamp_).seconds();
    last_stamp_ = now;
    if (dt > 1e-6)
    {
        const double fps_inst = 1.0 / dt;
        fps_ema_ = (fps_ema_ == 0.0) ? fps_inst : (fps_alpha_ * fps_inst + (1.0 - fps_alpha_) * fps_ema_);
    }

    // Raw data (a[m/s^2], w[rad/s])
    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;

    const double gx = msg->angular_velocity.x;
    const double gy = msg->angular_velocity.y;
    const double gz = msg->angular_velocity.z;

    // Stationary detection (for bias estimation)
    const double a_mag = norm3(ax, ay, az);
    const double w_mag = norm3(gx, gy, gz);
    const bool is_stationary = (std::fabs(a_mag - g_) < accel_tol_) && (w_mag < gyro_tol_);

    // Gyro bias estimation while stationary
    if (is_stationary && bias_count_ < bias_samples_needed_)
    {
        const double k = 1.0 / static_cast<double>(bias_count_ + 1);
        gbx_ = (1.0 - k) * gbx_ + k * gx;
        gby_ = (1.0 - k) * gby_ + k * gy;
        gbz_ = (1.0 - k) * gbz_ + k * gz;
        ++bias_count_;
    }

    // Bias removal + exponential filtering
    const double gx_nb = gx - gbx_;
    const double gy_nb = gy - gby_;
    const double gz_nb = gz - gbz_;

    ax_f_ = filt_alpha_ * ax + (1.0 - filt_alpha_) * ax_f_;
    ay_f_ = filt_alpha_ * ay + (1.0 - filt_alpha_) * ay_f_;
    az_f_ = filt_alpha_ * az + (1.0 - filt_alpha_) * az_f_;

    gx_f_ = filt_alpha_ * gx_nb + (1.0 - filt_alpha_) * gx_f_;
    gy_f_ = filt_alpha_ * gy_nb + (1.0 - filt_alpha_) * gy_f_;
    gz_f_ = filt_alpha_ * gz_nb + (1.0 - filt_alpha_) * gz_f_;

    // Convention (example): right-handed, x-forward, y-left, z-up (depends on IMU mounting)
    // roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2))
    const double roll_rad = std::atan2(ay_f_, az_f_);
    const double pitch_rad = std::atan2(-ax_f_, std::sqrt(ay_f_ * ay_f_ + az_f_ * az_f_));

    // print
    const double now_s = now.seconds();
    if (now_s - last_print_sec_ >= 1.0 / std::max(0.1, print_hz_))
    {
        last_print_sec_ = now_s;

        const double a_err = std::fabs(norm3(ax_f_, ay_f_, az_f_) - g_);
        const double w_deg = rad2deg(norm3(gx_f_, gy_f_, gz_f_));

        RCLCPP_INFO(this->get_logger(),
                    "[imu] FPS=%.1f | a(m/s^2)=(% .2f,% .2f,% .2f) | |a|-g=%.2f | "
                    "w(deg/s)=(% .2f,% .2f,% .2f) | |w|=%.2f | roll/pitch(deg)=(% .1f,% .1f) | "
                    "stationary=%s bias(%d/%d)",
                    fps_ema_,
                    ax_f_, ay_f_, az_f_, a_err,
                    rad2deg(gx_f_), rad2deg(gy_f_), rad2deg(gz_f_), w_deg,
                    rad2deg(roll_rad), rad2deg(pitch_rad),
                    is_stationary ? "Y" : "N",
                    bias_count_, bias_samples_needed_);
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
    rclcpp::spin(std::make_shared<LivoxImuLite>());
    rclcpp::shutdown();
    return 0;
}