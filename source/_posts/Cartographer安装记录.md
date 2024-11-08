---
title: Cartographer 记录
author: xmcchv
date: 2024-11-8 22:01:33
tags: 
- SLAM
- Cartographer
---

# Cartographer 记录

## 一、安装记录

1. 遇到abseil安装失败的情况下
    
    到/usr/local/lib  lib/pkgconfig  /usr/local/stow/absl 下把absl相关的文件全部删除
    
    再重新安装
    

1. 使用脚本自动安装和carto里面的script是一致的

![image.png](/images/Cartographer/image.png)

## 二、 运行记录

1. 运行纯laser scan建图
    
    首先编写launch文件 其中加载configuration_files路径下的lua配置文件
    
    其中的remap中的to话题需要更改为bag中的laser的topic
    
    ```bash
    <launch>
      <!-- bag的地址与名称 -->
      <!-- <arg name="bag_filename" default="$(env HOME)/bagfiles/rslidar-outdoor-gps-notf.bag"/> -->
      <arg name="bag_filename" default="$(env HOME)/bagfiles/slam-filtered.bag"/>
      
      <param name="/use_sim_time" value="true" />
    
      <node name="cartographer_node" pkg="cartographer_ros"
          type="cartographer_node" args="
              -configuration_directory $(find cartographer_ros)/configuration_files
              -configuration_basename sz.lua"
          output="screen">
        <remap from="scan" to="front_scan" />
      </node>
    
      <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
          type="cartographer_occupancy_grid_node" args="-resolution 0.05" />
    
      <node name="rviz" pkg="rviz" type="rviz" required="true"
          args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />
      <node name="playbag" pkg="rosbag" type="play"
          args="--clock $(arg bag_filename)" />
    </launch>
    ```
    
    然后是编写lua配置文件
    
    ```bash
    include "map_builder.lua"
    include "trajectory_builder.lua"
    
    options = {
      map_builder = MAP_BUILDER,
      trajectory_builder = TRAJECTORY_BUILDER,
      map_frame = "map",
      tracking_frame = "front_laser",
      published_frame = "front_laser",
      odom_frame = "odom",
      provide_odom_frame = true,
      publish_frame_projected_to_2d = false,
      use_pose_extrapolator = true,
      use_odometry = false,
      use_nav_sat = false,
      use_landmarks = false,
      num_laser_scans = 1,
      num_multi_echo_laser_scans = 0,
      num_subdivisions_per_laser_scan = 1,
      num_point_clouds = 0,
      lookup_transform_timeout_sec = 0.2,
      submap_publish_period_sec = 0.3,
      pose_publish_period_sec = 5e-3,
      trajectory_publish_period_sec = 30e-3,
      rangefinder_sampling_ratio = 1.,
      odometry_sampling_ratio = 1.,
      fixed_frame_pose_sampling_ratio = 1.,
      imu_sampling_ratio = 1.,
      landmarks_sampling_ratio = 1.,
    }
    
    MAP_BUILDER.use_trajectory_builder_2d = true
    
    TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
    TRAJECTORY_BUILDER_2D.min_range = 0.3
    TRAJECTORY_BUILDER_2D.max_range = 8.
    TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
    TRAJECTORY_BUILDER_2D.use_imu_data = false
    TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
    TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
    
    POSE_GRAPH.optimization_problem.huber_scale = 1e2
    POSE_GRAPH.optimize_every_n_nodes = 35
    POSE_GRAPH.constraint_builder.min_score = 0.65
    
    return options
    ```
    
    其中tracking_frame 指的是全部转到这个坐标系下，因为这里使用的是纯laser建图，如果有imu的话可以改成imu的frame_id，一般是imu_link
    
    published_frame指的是发布的话题tf: map -> front_laser
    

![image.png](/images/Cartographer/image%201.png)

附：

lua参数解析

https://blog.csdn.net/YiYeZhiNian/article/details/131774720

坐标系转换，rosbag

https://blog.51cto.com/u_14355665/6099741

cartographer保存2d地图

https://blog.csdn.net/jiesunliu3215/article/details/124359333