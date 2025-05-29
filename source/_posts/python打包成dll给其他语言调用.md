---
title: python打包成dll给其他语言调用
date: 2025-01-20 23:34:30
tags:
- python 
- dll
---

将 Python 代码打包成 DLL 供其他语言调用
以下是通过 Cython 将 Python 代码编译为 DLL 的完整流程，支持 C/C++、C# 等语言调用。此方法依赖 Python 运行时环境（目标机器需安装相同版本的 Python）。

### 步骤 1：安装依赖
```bash
pip install cython
```

### 步骤 2：创建示例 Python 代码
```bash
# mymodule.py
def add(a: int, b: int) -> int:
    return a + b

def greet(name: str) -> str:
    return f"Hello, {name}!"
```

### 步骤 3：创建 Cython 包装文件
```bash
# mymodule_wrapper.pyx
cdef public int add(int a, int b):
    return a + b

cdef public char* greet(const char* name):
    return name  # 简化示例（实际需处理字符串转换）
```

### 步骤 4：创建 Setup 脚本
```bash
# setup.py
from distutils.core import setup
from Cython.Build import cythonize

setup(
    name="MyModule",
    ext_modules=cythonize("mylib.pyx"),  # 编译包装文件
    script_args=["build_ext", "--inplace"]
)
```

### 步骤 5：编译生成 DLL
```bash
python setup.py build_ext --inplace
```
生成文件：mylib.dll（Windows）或 mylib.so（Linux）

### 在 C++ 中调用 DLL（示例）
```cpp
#include <iostream>
#include <Windows.h>

typedef int(*AddFunc)(int, int);
typedef const char*(*GreetFunc)(const char*);

int main() {
    HINSTANCE hDLL = LoadLibrary(TEXT("mylib.dll"));
    if (!hDLL) {
        std::cerr << "无法加载 DLL!" << std::endl;
        return 1;
    }

    // 获取函数指针
    AddFunc add = (AddFunc)GetProcAddress(hDLL, "add");
    GreetFunc greet = (GreetFunc)GetProcAddress(hDLL, "greet");

    if (add && greet) {
        std::cout << "加法结果: " << add(3, 5) << std::endl;
        std::cout << "问候语: " << greet("World") << std::endl;
    } else {
        std::cerr << "无法找到函数!" << std::endl;
    }

    FreeLibrary(hDLL);
    return 0;
}
```



