---
title: ROS2 Fast DDS 占用路由器上传带宽
date: 2025-12-23 15:50:08
mathjax: True
tags:
- FastDDS
- CycloneDDS
- ROS2
---

# ROS2 Fast DDS 占用路由器上传带宽

⭐原因：

ROS2默认的DDS中间件（如Fast DDS）会使用**多播(Multicast)**和**广播**进行节点自动发现，这些网络包会被路由器处理并统计为“上传流量”。

✨解决方案：

**1. 定制Fast DDS XML配置文件**，禁用内置传输、绑定网卡、并**将节点发现严格限制在指定的单播IP列表**

**关键配置**：

- `<useBuiltinTransports>false</useBuiltinTransports>`：**禁用默认传输**，这是关闭多播的关键。
- `<interfaceWhiteList>`：将传输绑定到**本机局域网IP**，防止从其他网络接口泄露流量。
- `<initialPeersList>`：将节点发现**严格限定在`192.168.0.200`至`.203`这四个单播地址**，替代了全网的广播/多播发现

```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <!-- 【第一步：必须先定义传输配置】 -->
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_v4_local</transport_id>
            <type>UDPv4</type>
            <!-- 可选：进一步限制发送的网卡IP，此处绑定到你的局域网IP -->
            <interfaceWhiteList>
                <address>192.168.0.201</address> <!-- 请改为本机IP -->
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>

    <!-- 【第二步：在参与者配置中引用定义好的传输】 -->
    <participant profile_name="ros2_local_network">
        <rtps>
            <!-- 1. 使用我们自定义的传输 -->
            <userTransports>
                <transport_id>udp_v4_local</transport_id>
            </userTransports>
            <!-- 禁用默认的内置传输，这是避免多播流量的关键 -->
            <useBuiltinTransports>false</useBuiltinTransports>

            <builtin>
                <!-- 2. 将发现严格限制在4台设备的单播地址 -->
                <initialPeersList>
                    <!-- 本机 -->
                    <locator>
                        <udpv4>
                            <address>192.168.0.201</address>
                            <port>11811</port>
                        </udpv4>
                    </locator>
                    <!-- 需要通信的其他三台设备 -->
                    <locator>
                        <udpv4>
                            <address>192.168.0.202</address>
                            <port>11811</port>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>192.168.0.203</address>
                            <port>11811</port>
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                            <address>192.168.0.200</address>
                            <port>11811</port>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
            
            <!-- 启用数据共享，并设置共享内存域大小 -->
            <datasharing>
                <kind>AUTO</kind> <!-- 或 EXPLICIT -->
                <shm_size>1048576</shm_size> <!-- 共享内存区域大小，单位字节，可调整 -->
            </datasharing>
        </rtps>
    </participant>
</profiles>
```

**2. 切换DDS中间件为CycloneDDS**

CycloneDDS默认不会使用多播进行节点发现，适合对网络流量敏感的应用场景。
可以通过以下命令安装CycloneDDS：

```bash
sudo apt install ros-<distro>-cyclonedds
```

然后在运行ROS2节点时，设置环境变量指定使用CycloneDDS：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

通过上述两种方法，可以有效避免ROS2 Fast DDS占用路由器上传带宽的问题。