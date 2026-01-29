#ifndef MOTOR_SDK_H
#define MOTOR_SDK_H

#include <thread>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "serial_port.h"
#include "UDPPort.h"
#include "absolute_timer.h"
#include "comm.h"
#include "yaml-cpp/yaml.h"

class MotorSDK{
public:
    MotorSDK(uint8_t &motor_num, uint8_t &imu_num, uint8_t &battery_num, std::vector<double> &offset, std::vector<double> &gr, std::vector<uint16_t> &id, CommType comm_type = CommType::UDP);
    MotorSDK();
    ~MotorSDK();

    void init(std::string yaml_path, CommType comm_type = CommType::UDP);

    void sendRecv(const std::vector<MotorCmd> &m_c_vec, std::vector<MotorState> &m_s_vec, std::vector<ImuState> &i_s_vec, std::vector<BatState> &b_s_vec);
    void recv(std::vector<MotorState> &m_s_vec, std::vector<ImuState> &i_s_vec, std::vector<BatState> &b_s_vec);
    void recvRaw(std::vector<MotorState> &m_s_vec, std::vector<ImuState> &i_s_vec, std::vector<BatState> &b_s_vec);
    void setMotorZero(const std::vector<MotorCmd> &m_c_vec, std::vector<MotorState> &m_s_vec, std::vector<ImuState> &i_s_vec, std::vector<BatState> &b_s_vec);

    void setSendMT(const std::vector<MotorCmd> &m_c_vec);
    void getRecvMT(std::vector<MotorState> &m_s_vec, std::vector<ImuState> &i_s_vec, std::vector<BatState> &b_s_vec);
    void sendRecvMT();
    void recvMT();

    void stop();
    void close();

    void print(const std::vector<MotorCmd> &m_c_vec, const std::vector<MotorState> &m_s_vec, const std::vector<ImuState> &i_s_vec, const std::vector<BatState> &b_s_vec);
    void print(const std::vector<MotorState> &m_s_vec, const std::vector<ImuState> &i_s_vec, const std::vector<BatState> &b_s_vec);

    uint8_t get_motor_num(){return motor_num_;}
    uint8_t get_imu_num(){return imu_num_;}
    uint8_t get_battery_num(){return battery_num_;}

    double get_motor_gr(int idx){return gr_.at(idx);}
    double get_motor_jpos_max(int idx){return jpos_max_.at(idx);}
    double get_motor_jpos_min(int idx){return jpos_min_.at(idx);}
    bool is_dual_encoder(){return is_dual_encoder_;}

    std::string get_version();

private:
    long long start_time{0};
    IOPort *sp_ctrl_ = nullptr;

    uint8_t motor_num_;
    uint8_t imu_num_;
    uint8_t battery_num_;
    std::vector<uint8_t> motor_type_; //    std::vector<MotorType> motor_type_;
    std::vector<uint16_t> id_;
    std::vector<double> offset_;
    std::vector<double> gr_;
    std::vector<double> jpos_min_;
    std::vector<double> jpos_max_;

    size_t m_c_len_;                        // motor command length
    size_t m_s_len_;                        // motor state length
    size_t i_s_len_;                        // imu state length
    size_t b_s_len_;                        // battery state length
    std::vector<double> current_pos_;       // output value of motor
    std::vector<MotorCmd> m_c_vec_;         // motor command vector : rotor value of motor
    std::vector<MotorState> m_s_vec_;       // motor state vector : rotor value of motor
    std::vector<ImuState> i_s_vec_;         // imu state vector
    std::vector<BatState> b_s_vec_;         // battery state vector

    bool is_dual_encoder_ = false;
};

#endif // MOTOR_SDK_H
