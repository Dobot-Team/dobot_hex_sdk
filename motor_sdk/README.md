# Motor SDK

## Introduction
This is a motor software development kit(brif. motor SDK) for INFFNI robotics, which can be used to control motors directly by your controller. 

## Getting started 
### Preparation
1. Turn on the robot
2. Connect the robot wifi
3. log in the robot by SSH command()
```
ssh robot@192.168.12.1
passwd: 123
```
4. kill robot motion control program
```
ps -ef | grep start_controller.sh
sudo kill <progress ID>
ps -ef | grep main
sudo kill <progress ID>
```

### Build
```
cd ~/motor_sdk
mkdir build
cmake ..
make
```

### Run 
```
sudo ./example
```
