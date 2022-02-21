---
title: Ubuntu20.04+noetic+ORB_SLAM2安装记录
date: 2022-02-21 14:31:13
author: xmcchv
tags: 
- ROS
- ORB_SLAM2
---

## Ubuntu20.04 安装 noetic
本段教程参考于 https://www.guyuehome.com/12640
1. 安装过程与ROS官网一致，网络无法连接时可以换个服务器,或使用国内镜像
    ![选择最快的服务器](/images/choose_best_server.png)
    安装完成后可以跑以下小乌龟的例子看一下是否ok


2. 安装完成后需要 sudo rosdep init ,提示website may be down ,总之在浏览器中可以打开这个网址，但终端中不能访问到，复制代码，新建文件，手动放到目录/etc/ros/rosdep/sources.list.d下

```bash
sudo rosdep init
rosdep update
```

![](/images/raw_githubcontent_rosdep_init.png)

    ```bash
    sudo mkdir -p /etc/ros/rosdep/sources.list.d
    cd /etc/ros/rosdep/sources.list.d
    sudo gedit 20-default.list
    ```

    ```bash
    # os-specific listings first
    yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

    # generic
    yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
    yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
    yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
    gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte

    # newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
    ```
    之后执行rosdep update
    若是githubusercontent不能访问，可以将其改为github



## 创建ROS工作空间

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make

# 在.bashrc 中添加
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```


## 编译usb_cam
[usb_cam下载链接] (https://github.com/bosch-ros-pkg/usb_cam)
在ros工作空间 src目录下进行
```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
cd usb_cam
mkdir build
cd build
cmake ..
make
```

## 编译安装ORB_SLAM2
本段教程参考自( https://www.cnblogs.com/MingruiYu/p/12286752.html )

1. Pangolin

    安装教程见github https://github.com/stevenlovegrove/Pangolin
    实测最新的pangolin需求c17过新 安装旧版V0.5就行

2. opencv

    到opencv官网 https://opencv.org/releases 下载源码自行编译安装
    下载后解压

    ```bash
    cd ~/opencv
    mkdir build	 # 创建工程编译所需文件夹
    cd build


    #注意，后面的两个点千万不能省，代表了上级目录
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..

    make -j10     # 多线程执行make任务
    sudo make install

    pkg-config --modversion opencv	#查看安装版本

    ```

3. Eigen

    其下载和安装教程可见：(http://eigen.tuxfamily.org)。在ubuntu中，可以直接使用 sudo apt-get install libeigen3-dev进行安装

    注：关于Eigen3的安装路径，如果出现程序include不到Eigen3的情况，可以参考 (https://www.cnblogs.com/newneul/p/8256803.html) 对Eigen3的位置进行调整。

4. 安装ORB_SLAM2
    ```bash
    # 在 /catkin_ws/src 目录下
    git clone https://github.com/raulmur/ORB_SLAM2.git
    cd ORB_SLAM2
    chmod +x build.sh
    ./build.sh
    ```

## 把摄像头数据发布到topic
```bash
cd ~/catkin_ws
# 若是提示没有usb_cam什么的 运行 source /devel/setup.bash 重试
roslaunch usb_cam usb_cam-test.launch
```

## 修改ros_mono.cc

进入～/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/src
打开 ros_mono.cc 把程序里面的topic改为 /usb_cam/image_raw
具体还要查看自己的rviz（即上一步打开的摄像头窗口的标题）
![](/images/ros_mono_usb_cam.png)

## 编译ros下的ORB_SLAM2
```bash
echo $ROS_PACKAGE_PATH
# 使用echo来检查ros package path 是否配置
# 在~/.bashrc 中添加 
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/自己的用户名/catkin_ws/src/ORB_SLAM2/Examples/ROS


#  然后进入～/catkin_ws/src/ORB_SLAM
chmod +x build_ros.sh
./build_ros.sh
```

## 启动mono
```shell
# 在目录~/catkin_ws/src/ORB_SLAM2下
rosrun ORB_SLAM2 Mono /home/xmcchv/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt /home/xmcchv/catkin_ws/src/ORB_SLAM2/myUsb_cam.yaml

```
有两个参数，第二个参数是相机参数文件，根据自己的相机参数修改。
启动后正常如图
![](/images/ROS_ORB_SLAM2.png)



## 相机标定 获取参数
本段教程参考于( https://blog.csdn.net/yx868yx/article/details/115366655 )
使用棋盘格 

![](/images/qipange.png)

1. 下载ros的相机标定包
    ```bash
    rosdep install camera_calibration
    rosmake camera_calibration
    ```

2. 启动usb_cam
    ```bash
    # 若失败 source一下 devel/setup.bash
    roslaunch usb_cam usb_cam-test.launch
    ```
3. 启动标定程序
    ```bash
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam
    ```
    其中image:= 和camera:=后的内容需要根据自己电脑摄像头的话题和节点来修改，可以通过rqt_graph来查看。比如有的会是image:=/camera/image_raw camera:=/camera


    标定界面出现后，按照x（左右）、y（上下）、size（前后）、skew（倾斜）等方式移动棋盘，直到x,y,size,skew的进度条都变成绿色位置．

    此时可以按下CALIBRATE按钮，点击一下后，界面会卡住,此时不要做任何操作，直到运行标定程序的终端输出标定的结果
    有标定结果出来后，点击标定界面的SAVE按钮，再点commit按钮，标定结果保存在/tmp/calibrationdata.tar.gz这个压缩包中，到这里彩色相机的标定就结束了
    关闭标定程序,完成后Commit，在终端后会有标定结果yaml文件地址．默认一般为/home/用户名/.ros/camera_info/head_camera.yaml

    ![](/images/cam_yaml.png)

4. 修改相机名称

    如图
![](/images/usb_cam-test_launch.png)
    修改/dev/video0 和 usb_cam 
    与其他地方保持一致

## 参考链接

总体安装教程

https://blog.csdn.net/xmy306538517/article/details/59501718
http://www.liuxiao.org/2016/07/ubuntu-orb-slam2-%E5%9C%A8-ros-%E4%B8%8A%E7%BC%96%E8%AF%91%E8%B0%83%E8%AF%95/
https://blog.csdn.net/qinqinxiansheng/article/details/107079265

编译build_ros.sh时报错 
https://blog.csdn.net/weixin_41111088/article/details/89377368

ORB_SLAM2编译build_ros.sh时报错
https://blog.csdn.net/chengmo123/article/details/104906376

相机标定
https://blog.csdn.net/yx868yx/article/details/115366655

ORB_SLAM2数据集
https://vision.in.tum.de/data/datasets/rgbd-dataset/download#freiburg2_pioneer_360

ORB_SLAM2编译build.sh 出错
https://blog.csdn.net/lixujie666/article/details/90023059


小乌龟
https://blog.csdn.net/qq_21835111/article/details/99694286









