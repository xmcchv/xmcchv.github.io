---
title: Webviz安装记录
date: 2024-10-25 19:34:09 
tags:
- web
- ROS
---

# webviz

- docker 镜像拉取
  - 配置daemon.json 

    ```
    sudo gedit /etc/docker/daemon.json

    # 内容
    {
        "runtimes": {
            "nvidia": {
                "path": "nvidia-container-runtime",
                "runtimeArgs": []
            }
        },
        "registry-mirrors": [
            "https://mirror.tuna.tsinghua.edu.cn/docker/",
            "http://hub-mirror.c.163.com",
            "https://reg-mirror.com",
            "https://docker.registry.cyou",
            "https://docker-cf.registry.cyou",
            "https://dockercf.jsdelivr.fyi",
            "https://docker.jsdelivr.fyi",
            "https://dockertest.jsdelivr.fyi",
            "https://mirror.aliyuncs.com",
            "https://dockerproxy.com",
            "https://mirror.baidubce.com",
            "https://docker.m.daocloud.io",
            "https://docker.nju.edu.cn",
            "https://docker.mirrors.sjtug.sjtu.edu.cn",
            "https://docker.mirrors.ustc.edu.cn",
            "https://mirror.iscas.ac.cn",
            "https://docker.rainbond.cc"
        ]
    }

    # 重启docker

    sudo systemctl daemon-reload
    sudo systemctl restart docker
    ```


  - docker run -p 8080:8080 cruise/webviz

- 启动rosbridge
 
  - 首先安装rosbridge

    ```
    sudo apt install ros-noetic-rosbridge-suite
    ```
  - 启动
    
    ```
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```


- 启动ros程序，确保话题发布到ros中

- 访问webviz
  - 本地访问

    打开ws://localhost:8080

  - 局域网内访问

    使用ip访问
    ```
    192.168.110.158:8080/?rosbridge-websocket-url=ws://192.168.110.158:9090
    ```  


- 参考链接

  - https://www.codetd.com/en/article/13441634
  - https://blog.csdn.net/Adoreuu/article/details/138925628
  - https://blog.csdn.net/Adoreuu/article/details/138925628
  - https://webviz.io/app/?demo