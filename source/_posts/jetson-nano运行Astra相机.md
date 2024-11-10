---
title: jetson nano运行Astra相机
date: 2024-11-10 22:57:16
tags:
- SLAM
- RGBD相机
- 奥比中光
---

## Astra相机

使用ros-astra-camera包

1. 安装libuvc
```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
```


2. 安装udev
```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd astra_camera
./scripts/create_udev_rules
sudo udevadm control --reload && sudo  udevadm trigger
```


3. 编译运行
```bash
catkin_make
source devel/setup.bash
roslaunch astra_camera astra.launch
```



## 运行报错

启动相机能够正常启动，但是一到打开depth就报错

![image.png](/images/astra_error.png)

原因是同目录下需要额外的ros_bridge一起编译，使ros环境不使用自带cv_bridge

https://github.com/orbbec/ros_astra_camera/issues/189#issuecomment-1687006729
