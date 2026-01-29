#ifndef ROBOT_SDK_H
#define ROBOT_SDK_H

#include <eigen3/Eigen/Dense>
#include "motor_sdk.h"
#include "periodic_function.h"

using Vec3d = Eigen::Matrix<double, 3, 1>;
using Vec4d = Eigen::Matrix<double, 4, 1>;
using DVecd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using DMatd = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

struct RobotState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotState() {}

    void zero(int nj){
        q.setZero(nj);
        qd.setZero(nj);
        tau.setZero(nj);

        qRaw.setZero(nj);
        qdRaw.setZero(nj);
        tauRaw.setZero(nj);
    }
    Vec3d acc, gyro;
    Vec4d quat;
    DVecd q, qd, tau;
    DVecd qRaw,qdRaw,tauRaw;
    bool isUpdated = false;
};

struct RobotCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotCommand() {}

    void zero(int nj){
        qDes.setZero(nj);
        qdDes.setZero(nj);
        tauDes.setZero(nj);
        kpDes.setZero(nj);
        kdDes.setZero(nj);
        is_enabled = false;
        is_exit = false;
    }

    DVecd tauDes, qDes, qdDes;
    DVecd kpDes, kdDes;


    bool is_send;
    bool is_enabled;
    bool is_exit;
};


class RobotSDK {
 public:
  RobotSDK(){};
  RobotSDK(std::string config_path, double dt,
           std::function<void(const RobotState& state, RobotCommand& command)> func);

  RobotSDK(std::string config_path);

  void init(double dt, std::function<void(const RobotState& state, RobotCommand& command)> func);
  void init();
  void start();
  void stop();
  int get_motor_num(){return _motor_sdk.get_motor_num();}
  void setCommand(const RobotCommand& command);
  void brake();
  void getState(RobotState& state);
  std::string get_version();

private:
  MotorSDK _motor_sdk;
  RobotCommand _command;
  RobotState _state;
  RobotCommand _command_hd;
  RobotState _state_hd;

  PeriodicFunction *comm;
  PeriodicFunction *control;
  PeriodicFunction *watch_dog;
  double _dt;

  std::vector<MotorCmd> _motor_command;
  std::vector<MotorState> _motor_state;
  std::vector<ImuState> _imu_state;
  std::vector<BatState> _battery_state;

  std::function<void(const RobotState& state, RobotCommand& command)> _controlFunc;

  int  comm_failure_count = 0;

  void _run();
  void _lowLevelComm();
  void _watchDog();
  void _setCommand();
  void _getState();

  int _robot_type;

  bool _flag_use_periodic_control = false;

};

#endif // ROBOT_CONTROLLER_H
