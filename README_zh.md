<p align="center">
  <img src="assets/hexplorer_logo.svg" alt="Hexplorer logo" height="160">
</p>

<p align="center">
  <img src="assets/hexplorer_model.svg" alt="Hexplorer model" width="320">
</p>

本仓库为 Dobot Hexplorer 六足机器人的关节电机 SDK 以及二次开发案例程序，包含三个高层控制案例、两个关节控制案例以及深度相机和激光雷达的示例程序。

支持型号：miniHex_v2

**准备工作**：

```bash
source /opt/ros/humble/setup.bash
source ~/robot_controller_release/ros2_packages/setup.bash
```

## 高层控制示例

进入高层控制案例目录(`high_level`)目录下，编译程序：

```bash
mkdir build && cd build
cmake ..
make
```

运行程序

```bash
./<executable file>
```

如果您需要运行 python 案例，请进入`py`目录中并在此执行

```bash
python3 <example file>.py
```

正常情况下，您应该看到如下几种情况，从左往右分别表示

- 身体扭动案例
- 行走案例
- 机器人状态控制台输出
- 机器人状态实机测试

<div align="center">
<table>
  <tr>
    <td align="center">
      <img src="assets/body_twist.gif" alt="body_twist" width="360" ><br>
      <b>body_twist</b>
    </td>
    <td align="center">
      <img src="assets/locomotion.gif" alt="locomotion" width="360"><br>
      <b>locomotion</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="assets/robot_state_console.gif" alt="robot_state_console" width="360" ><br>
      <b>robot_state 控制台读数</b>
    </td>
    <td align="center">
      <img src="assets/robot_state_real.gif" alt="robot_state_real" width="360" ><br>
      <b>robot_state 实机</b>
    </td>
  </tr>
</table>
</div>

## 关节控制示例

案例程序主要展示通过 robot interface api 与关节电机进行通信，包括从关节电机读取数据和发送数据。

**编译**

```bash
cd motor_sdk/
mkdir build && cd build
cmake ..
make
```

**停止控制程序**

防止自启动的控制程序与关节控制程序产生冲突，需要先将`start_controller.sh`和`main`进程杀掉

```bash
ps -ef | grep start_controller.sh
sudo kill <progress ID>
ps -ef | grep main
sudo kill <progress ID>
```

**运行**

```bash
cd build
./motor_read miniHex_v2
./motor_wave miniHex_v2
```

<div align="center">
  <img src="assets/motor_wave.gif" alt="motor_wave" width="420"><br>
  <b>motor_wave</b>
</div>

## 激光雷达

首先需要开启激光雷达节点，请确保在 **intel-minipc** 而不是 jetson-nano 上运行命令：

```bash
sudo chmod +x ros2_setup.bash
./ros2_setup.bash
ros2 launch livox_lidar_node start_node.launch.py
```

可通过`ros2 topic list`检验节点是否启动成功

**编译**

```bash
cd livox
mkdir build && cd build
cmake ..
make
```

**运行**

```bash
<executable file> # C++方式 或者
python3 ./py/<script file>  # python方式
```

若程序正常运行，用户应该可以看到控制台输出点云帧率(10hz)、数量以及 imu 的相关信息

## 深度相机

首先需要开启激光雷达节点，请确保在 **jetson-nano** 而不是 intel-minipc 上运行命令：

```bash
ssh robot@192.168.1.20  # 密码123
cd Hexplorer_sdk_examples
sudo chmod +x ros2_setup.bash
./ros2_setup.bash
ros2 launch realsense_camera_node start_node.launch.py
```

可通过`ros2 topic list`检验节点是否启动成功

**编译**

```bash
cd realsense
mkdir build && cd build
cmake ..
make
```

**运行**

```bash
<executable file> # C++方式 或者
python3 ./py/<script file>  # python方式
```

若程序正常运行，用户应该可以看到控制台输出相机的帧率(30hz)、尺寸等信息
