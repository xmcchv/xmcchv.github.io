---
title: cmake编译
date: 2022-02-17 12:06:24
author: xmcchv
tags: 
- cmake
---


## CMakeList.txt

记录下CMakeList中包含的内容

```
# 声明cmake的最低版本
cmake_minimum_version( VERSION 2.0 )

# 声明一个cmake工程
project( HelloSLAM )

# 添加一个可执行程序
# 语法 add_executable( 程序名 源代码文件 )
add_executable( helloSLAM helloSLAM.cpp )

# 编写库
# 静态库 .a 静态库每次被调用都会生成一个副本
add_library( hello libHelloSLAM.cpp )

# 共享库 .so 共享库只有一个副本
add_library( hello_shared SHARED libHelloSLAM.cpp )

# 使用 .so 时 还需要定义一个头文件 提供了共享库的函数 以供调用
# 链接 
target_link_libraries( useHello hello_shared )

# 编译
# 在源文件同级目录下 mkdir build && cd build && cmake ..

# 添加头文件
include_directories("/user/include/eigen3")

# 寻找Opencv库
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries( 可执行程序名 ${OpenCV_LIBS} )

```



