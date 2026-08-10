<p align="center">
  <img src="assets/hexplorer_logo.svg" alt="Hexplorer logo" height="160">
</p>

<p align="center">
  <img src="assets/hexplorer_model.svg" alt="Hexplorer model" width="320">
</p>

<p align="center">
  <a href="./README_zh.md">
    <img src="https://img.shields.io/badge/README-中文-blue">
  </a>
  <a href="./LICENSE">
    <img src="https://img.shields.io/badge/LICENSE-MIT-green">
  </a>
  <a href="https://www.dobot-robots.com/products">
    <img src="https://img.shields.io/badge/Homepage-web-orange">
  </a>
</p>

This repository contains the joint motor SDK and joint-level control examples for the **Dobot Hexplorer hexapod robot**.

Supported model: **miniHex_v2**

📖 **Secondary development documentation** — high-level control topics, LiDAR, depth camera, motor interface and system upgrade — see [wiki.md](./wiki.md).

## Joint Control Examples

These examples demonstrate how to communicate with the joint motors via the robot interface API,
including reading and sending data from/to the joint motors.

**Build**

```bash
cd motor_sdk/
mkdir build && cd build
cmake ..
make
```

**Stop existing control programs**

To avoid conflicts with the auto-start control program, first kill the `start_controller.sh` and `main` processes:

```bash
ps -ef | grep start_controller.sh
sudo kill <process ID>
ps -ef | grep main
sudo kill <process ID>
```

**Run**

```bash
cd build
./motor_read miniHex_v2
./motor_wave miniHex_v2
```

<div align="center">
  <img src="assets/motor_wave.gif" alt="motor_wave" width="420"><br>
  <b>motor_wave</b>
</div>
