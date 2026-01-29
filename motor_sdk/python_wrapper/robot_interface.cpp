#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <cmath>

#include "robot_sdk.h"

namespace py = pybind11;

class RobotInterface{
public:
    RobotInterface(){
        _robot_sdk = new RobotSDK();
    }

    RobotInterface(std::string config_path) : _config_path(config_path){
        _robot_sdk = new RobotSDK(_config_path);
        _robot_sdk->init();
        _robot_sdk->start();
        _motor_num = _robot_sdk->get_motor_num();
        _command.zero(_motor_num);
        _state.zero(_motor_num);
    }

    ~RobotInterface(){
        delete _robot_sdk;
    }

    std::vector<double> ReceiveObservation() {
        _robot_sdk->getState(_state);

        std::vector<double> obs;
        for(size_t j(0); j<_motor_num; ++j){
            obs.push_back(_state.q[j]);
            obs.push_back(_state.qd[j]);
            obs.push_back(_state.tau[j]);
        }
        obs.push_back(_state.acc[0]);
        obs.push_back(_state.acc[1]);
        obs.push_back(_state.acc[2]);
        obs.push_back(_state.gyro[0]);
        obs.push_back(_state.gyro[1]);
        obs.push_back(_state.gyro[2]);
        obs.push_back(_state.quat[0]);
        obs.push_back(_state.quat[1]);
        obs.push_back(_state.quat[2]);
        obs.push_back(_state.quat[3]);
        return obs;
    }

    void SendCommand(std::vector<double>& kpDes, std::vector<double>& kdDes, 
                     std::vector<double>& qDes, std::vector<double>& qdDes, std::vector<double>& tauDes) {
        for (size_t i = 0; i < _motor_num; ++i) {
            _command.kpDes[i] = kpDes[i];
            _command.kdDes[i] = kdDes[i];
            _command.qDes[i] = qDes[i];
            _command.qdDes[i] = qdDes[i];
            _command.tauDes[i] = tauDes[i];
        }
        _robot_sdk->setCommand(_command);
    }

    void Brake() {
        for (size_t i = 0; i < _motor_num; ++i) {
            _command.kpDes[i] = 0;
            _command.kdDes[i] = 0.5;
            _command.qDes[i] = 0.0;
            _command.qdDes[i] = 0.0;
            _command.tauDes[i] = 0.0;
        }
        _robot_sdk->setCommand(_command);
    }

    int get_motor_num() {
        int motor_num = 0;
        motor_num = _robot_sdk->get_motor_num();
        return motor_num;
    }

    std::string get_version() {
        return _robot_sdk->get_version();
    }

    std::string _config_path;

    RobotState _state;
    RobotCommand _command;

    RobotSDK* _robot_sdk;
    int _motor_num;

};

PYBIND11_MODULE(robot_interface, m) {
    py::class_<RobotInterface>(m, "RobotInterface")
        .def(py::init())
        .def(py::init<std::string>())
        .def("receive_observation", &RobotInterface::ReceiveObservation)
        .def("send_command", &RobotInterface::SendCommand)
        .def("brake", &RobotInterface::Brake)
        .def("get_motor_num", &RobotInterface::get_motor_num)
        .def("get_version", &RobotInterface::get_version);
}
