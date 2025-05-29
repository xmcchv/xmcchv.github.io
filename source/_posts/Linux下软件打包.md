---
title: Linux下软件打包
author: xmcchv
date: 2024-10-30 19:33:06
tags: 
- Linux
- AppImage
---

## Linux 将Qt程序打包为AppImage包

https://blog.csdn.net/no_say_you_know/article/details/134212620


### 1. 使用Linuxdeployqt工具打包qt

https://github.com/probonopd/linuxdeployqt

./linuxdeployqt /程序路径/程序  -appimage

### 2. 修改配置

修改desktop文件名字和内容。
```
[Desktop Entry]
Version=1.0
Name=xxx
Comment=xxx
Exec=xxx
Icon=xxx
Terminal=false
Type=Application
Categories=Application;
```

### 3. 打包成appimage

https://github.com/AppImage/AppImageKit

./appimagetool /程序所在文件夹路径
生成的Appimage在appimagetool的同级目录下

分发时，需要授予Appimage可执行权限。
