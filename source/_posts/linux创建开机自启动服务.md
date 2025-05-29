---
title: linux创建开机自启动服务
date: 2025-05-29 14:08:32
tags:
---

在 Ubuntu 中创建开机启动的 service 通常使用 systemd。以下是详细步骤：

1. 创建 service 文件
在 /etc/systemd/system/ 目录下创建 .service 文件（需 sudo 权限）：
```bash
sudo nano /etc/systemd/system/my_service.service
```

2. 编辑 service 文件
在 service 文件中，编写以下内容：
```bash
[Unit]
Description=My Service
After=network.target
[Service]
ExecStart=/path/to/your/script.sh
Restart=always
User=your_username
[Install]
WantedBy=multi-user.target
```
3. 解释 service 文件内容
- [Unit] 部分：
  - Description：描述服务的用途。
  - After：指定服务依赖的其他服务，这里是网络服务。
- [Service] 部分：
  - ExecStart：指定服务启动时运行的命令。
  - Restart：指定服务重启策略，这里设置为 always。
  - User：指定服务运行的用户。
- [Install] 部分：
  - WantedBy：指定服务在多用户环境下的启动方式。
4. 保存并退出
保存并退出 nano 编辑器。
5. 重新加载 systemd
```bash
sudo systemctl daemon-reload
```
6. 启用服务
```bash
sudo systemctl enable my_service    
```
7. 启动服务
```bash
sudo systemctl start my_service
```
8. 检查服务状态
```bash
sudo systemctl status my_service
```
9. 停止服务
```bash
sudo systemctl stop my_service
```
10. 禁用服务
```bash
sudo systemctl disable my_service
```
11. 查看服务日志
```bash
sudo journalctl -u my_service
```
