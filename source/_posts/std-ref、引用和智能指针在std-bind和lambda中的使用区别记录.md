---
title: std::ref、引用和智能指针在std::bind和lambda中的使用区别记录
date: 2025-08-02 22:08:35
tags:
- C++
---

## 一、引言
&emsp;&emsp;本人在写代码的时候用到这方面的内容，具体情况是有四个雷达和相机融合的数据时间同步的进到callback，然后要同时处理雷达点云的转换，以及雷达点到图像上的映射。我的写法使用了线程池，将处理过程写成lambda函数添加进线程池任务。但在我写的时候遇到了ref,取地址，智能指针在里面该怎么用的疑惑，记录一下。
 

 1. 雷达点云转换
这个比较简单就是，ros的convertmsg，我这是还有其他处理，再封了一层函数。
```cpp
std::future<pcl::PointCloud<pcl::PointXYZI>::Ptr> mfuture1 = mthreadPool->enqueue([this,pcl1]() -> pcl::PointCloud<pcl::PointXYZI>::Ptr{return this->ConvertLidarMsg(this->xyzi1, this->mLidarQueue1, pcl1);});
std::future<pcl::PointCloud<pcl::PointXYZI>::Ptr> mfuture2 = mthreadPool->enqueue([this,pcl2]() -> pcl::PointCloud<pcl::PointXYZI>::Ptr{return this->ConvertLidarMsg(this->xyzi2, this->mLidarQueue2, pcl2);});
std::future<pcl::PointCloud<pcl::PointXYZI>::Ptr> mfuture3 = mthreadPool->enqueue([this,pcl3]() -> pcl::PointCloud<pcl::PointXYZI>::Ptr{return this->ConvertLidarMsg(this->xyzi3, this->mLidarQueue3, pcl3);});
std::future<pcl::PointCloud<pcl::PointXYZI>::Ptr> mfuture4 = mthreadPool->enqueue([this,pcl4]() -> pcl::PointCloud<pcl::PointXYZI>::Ptr{return this->ConvertLidarMsg(this->xyzi4, this->mLidarQueue4, pcl4);});
```
这里传入this即可在lambda函数体中使用当前类中的成员变量了。然后pcd1是一个智能指针，所以直接传即可。如果是其他变量的话就会默认拷贝进去，这个地方不当心就会出浅拷贝问题。

 2. 雷达到图像的映射
这一块对每张图要做语义分割，然后转成灰度图；将雷达点映射到灰度图上作为该点的语义信息；转换的点云还要经过一个变换矩阵统一到一个坐标系中。
```cpp
std::future<void> fusiontask4 = mthreadPool->enqueue([this]() -> void {
   cv::Mat segmap4 = this->mpSegment->createSegmentationMap(this->mpSegment->vecResult[3], cv::Size(this->rgb4.cols, this->rgb4.rows));
   this->pcd4 = this->GeneratePointCloudinLidar(segmap4, this->pcd4,
       this->mvLidarDeviceList[3]->GetIntrisicMatrix(), this->mvLidarDeviceList[3]->GetDistortMatrix(),this->mvLidarDeviceList[3]->GetExtrisicMatrix());
   Converter::transformXYZIPointCloud(this->pcd4, this->Twc4);
});
```
这个就比较简单了，看起来代码多，其实传入this之后，内部全部使用this指针即可。

## 二、在 std::bind、Lambda、普通函数中的使用区别

以下是三者核心区别的对比表格：

| 特性 |	std::ref（引用包装）|	原生引用 T& |	std::shared_ptr<T>	| std::unique_ptr<T> |
|--|--|--|--|--|
| 本质 | 生成 reference_wrapper 对象	| 变量的别名 |	共享所有权的智能指针 |	独占所有权的智能指针 |
| 所有权语义 |	❌ 无所有权 |	❌ 无所有权 |		✅ 共享所有权 |		✅ 独占所有权 |	
 |	能否为空 (Nullable)	 |	❌ 不能（必须绑定对象） |		❌ 不能	 |	✅ 可以 (nullptr) |		✅ 可以 (nullptr) |	
 |	线程安全性 |		❌ 需手动同步	 |	❌ 需手动同步 |		✅ 引用计数线程安全 |		❌ 需手动转移所有权 |	
 |	生命周期管理	 |	❌ 依赖外部对象 |		❌ 依赖外部对象	 |	✅ 自动释放	 |	✅ 自动释放 |	
 |	重新绑定	 |	❌ 不能 |		❌ 不能 |		✅ 可重置指向新对象	 |	✅ 可移动转移所有权 |	
 |	典型用途 |		1. std::bind 保持引用 <br> 2. 存储引用到容器	 |	函数参数/返回值优化 |		跨线程共享数据	 |	独占资源管理 |	
 |	跨线程传递 |		❌ 不安全 |		❌ 不安全 |		✅ 安全 |		❌ 需移动语义 |	
 |	延长对象生命周期 |		❌ 不能 |		❌ 不能 |		✅ 能	 |	✅ 能（但独占） |	
 |	修改原对象 |		✅ 能 |		✅ 能 |		✅ 能 |		✅ 能 |	

### 1. std::ref 的使用场景
```cpp
void foo(int& x);
int a = 10;

// 普通函数调用无需包装
foo(std::ref(a));  // 错误：不能将 reference_wrapper 传给 int&

auto task = std::bind(update, val); // 无效
task(); // val 仍为 0

// std::bind 必须显式包装引用
auto task = std::bind(foo, std::ref(a)); 
task(); // 修改原变量 a

// Lambda 中通常不需要（优先用 [&]）
auto lambda = [&a]() { foo(a); }; 
```
### 2. 智能指针的使用场景

```cpp
auto ptr = std::make_shared<int>(42);

// Lambda 捕获智能指针（共享所有权）
auto lambda = [ptr]() { 
    std::cout << *ptr; // 安全：ptr 的生命周期由引用计数保证
};

// std::bind 直接传递（自动处理所有权）
auto task = std::bind([](std::shared_ptr<int> p) {}, ptr);
```
### 3. 原生引用的限制
```cpp
int a = 10;
int& ref = a;

// 错误示例：跨线程传递原生引用
std::thread([&ref]() { ref++; }).detach(); // ❌ 危险！可能悬垂引用

// 正确做法：改用智能指针
auto shared_a = std::make_shared<int>(a);
std::thread([shared_a]() { (*shared_a)++; }).join(); // ✅ 安全
```

## 三、Lambda 函数中传递不同类型值的几种方法总结
1. 按值捕获（[=] 或 [var]）
适用类型：基本类型（int, float）、小型结构体、需要副本的对象。
特点：创建变量的副本，Lambda 内修改不影响外部变量。
```cpp
int x = 10;
auto lambda = [x]() { 
    // 捕获的是 x 的副本， mutable 修改的也是副本
    std::cout << x; 
};
```

2. 按引用捕获（[&] 或 [&var]）
适用类型：大型对象（如 cv::Mat）、需要修改外部变量时。
特点：直接引用外部变量，无拷贝开销，但需确保 Lambda 执行时变量仍有效。
```cpp
std::vector<int> data = {1, 2, 3};
auto lambda = [&data]() { 
    data.push_back(4); // 修改原数据
};
```

3. 捕获智能指针（std::shared_ptr/std::unique_ptr）
适用类型：需要跨线程共享或管理动态对象的生命周期。
特点：自动管理内存，避免悬垂指针。

```cpp
auto ptr = std::make_shared<int>(42);
auto lambda = [ptr]() { 
    std::cout << *ptr; // 安全：ptr 的引用计数增加
};
// std::unique_ptr 需移动捕获（所有权转移）
auto uptr = std::make_unique<int>(100);
auto lambda = [uptr = std::move(uptr)]() { /* 使用 uptr */ };
```

4. 捕获 std::ref 包装的引用
适用场景：需要将引用存储到容器或延迟调用（如 std::bind）。
特点：显式包装引用，但生命周期仍需手动管理。
```cpp
int a = 10;
auto ref_a = std::ref(a);
auto lambda = [ref_a]() { 
    ref_a.get()++; // 修改原变量 a
};
```

5. 捕获成员变量（通过 this 或局部引用）
适用类型：类成员变量。
特点：需通过 this 或局部变量间接捕获。
```cpp
class MyClass {
    int value = 100;
public:
    auto getLambda() {
        return [this]() { std::cout << value; }; // 捕获 this
        // 或通过局部引用：
        // int& ref_val = value;
        // return [&ref_val]() { std::cout << ref_val; };
    }
};
```
这里的value编译器会自动写成this->value，根据c++命名查找规则，只写value时会优先使用局部变量。代码如下：

```cpp
 auto getLambda() {
        int value = 10;
        return [this]() { 
        	std::cout << value; // value 10
        	std::cout << this->value; 
        }; 
    }
```
