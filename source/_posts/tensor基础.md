---
title: tensor基础
author: xmcchv
date: 2021-07-24 17:39:29
tags:
- pytorch
---
## 张量的创建与常用方法


```python
import numpy as np
import torch
```


```python
a1=np.array([[1,2,3],[4,5,6]])
a1
```




    array([[1, 2, 3],
           [4, 5, 6]])




```python
t3=torch.tensor([a1,a1])
t3
```




    tensor([[[1, 2, 3],
             [4, 5, 6]],
    
            [[1, 2, 3],
             [4, 5, 6]]], dtype=torch.int32)




```python
t3.size()
```




    torch.Size([2, 2, 3])




```python
t3.ndim
```




    3




```python
len(t3)
```




    2




```python
t3.flatten()
```




    tensor([1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6], dtype=torch.int32)




```python
t1=torch.tensor([1,2])
t1
```




    tensor([1, 2])




```python
t1.shape
```




    torch.Size([2])




```python
# 转化为两行，一列的张量
t1.reshape(2,1)
```




    tensor([[1],
            [2]])




```python
t1.reshape(2)
```




    tensor([1, 2])




```python
t2=torch.tensor([1,2,3])
t2
```




    tensor([1, 2, 3])




```python
t2.reshape([3,1])
```




    tensor([[1],
            [2],
            [3]])




```python
t2.reshape(3)
```




    tensor([1, 2, 3])




```python
t3.reshape(12)
```




    tensor([1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6], dtype=torch.int32)




```python
t3.reshape(2,6) # 转换为两行六列的张量
```




    tensor([[1, 2, 3, 4, 5, 6],
            [1, 2, 3, 4, 5, 6]], dtype=torch.int32)




```python
torch.ones([2,3])
```




    tensor([[1., 1., 1.],
            [1., 1., 1.]])




```python
torch.eye(5)  # 单位矩阵
```




    tensor([[1., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0.],
            [0., 0., 1., 0., 0.],
            [0., 0., 0., 1., 0.],
            [0., 0., 0., 0., 1.]])




```python
t1
torch.diag(t1) # 不能使用list直接创建对角矩阵
```




    tensor([[1, 0],
            [0, 2]])




```python
torch.rand(2,3) # rand服从0-1均匀分布
```




    tensor([[0.9925, 0.4613, 0.1272],
            [0.6944, 0.3979, 0.4879]])




```python
torch.randn(2,3) # 服从标准正态分布的张量
```




    tensor([[ 0.7202, -1.3448,  1.1586],
            [-1.3567, -0.6265,  0.6884]])




```python
torch.normal(2,3,size=(2,2)) # 均值为2，标准差为3的张量  指定正态分布
```




    tensor([[-1.8283,  0.6573],
            [ 0.7957,  4.2425]])




```python
torch.randint(1,10,[2,4]) # 在1-10之间随机抽取整数，组成两行四列的张量
```




    tensor([[2, 8, 1, 3],
            [4, 6, 9, 5]])




```python
torch.arange(5)  # 默认每隔1取
```




    tensor([0, 1, 2, 3, 4])




```python
torch.arange(1,5,0.5) # 从1-5 左闭右开 每隔0.5取值一个
```




    tensor([1.0000, 1.5000, 2.0000, 2.5000, 3.0000, 3.5000, 4.0000, 4.5000])




```python
torch.linspace(1,5,3) # 从1-5 闭区间 等距取三个数
```




    tensor([1., 3., 5.])




```python
torch.linspace(1,9,6)
```




    tensor([1.0000, 2.6000, 4.2000, 5.8000, 7.4000, 9.0000])




```python
torch.empty(2,3)
```




    tensor([[8.1717e+20, 2.1274e+23, 8.5032e+20],
            [1.0616e+21, 4.3445e-05, 2.0432e+20]])




```python
torch.full([2,4],2)  # 两行四列 填充2 的张量
```




    tensor([[2, 2, 2, 2],
            [2, 2, 2, 2]])




```python
torch.full_like(t1,2)
```




    tensor([2, 2])




```python
torch.randint_like(t2,1,10)
```




    tensor([5, 1, 5])




```python
t1=torch.arange(1,11)
t1
```




    tensor([ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10])




```python
t1.numpy()
```




    array([ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10], dtype=int64)




```python
np.array(t1)
```




    array([ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10], dtype=int64)




```python
t1.tolist()
```




    [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]




```python
list(t1)   # 一维张量由零维张量组成 二维由一维组成
```




    [tensor(1),
     tensor(2),
     tensor(3),
     tensor(4),
     tensor(5),
     tensor(6),
     tensor(7),
     tensor(8),
     tensor(9),
     tensor(10)]




```python
t1
```




    tensor([ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10])




```python
t11=t1.clone()
t11
```




    tensor([ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10])




```python
t1[0]=200
print(t1)
print(t11)
```

    tensor([200,   2,   3,   4,   5,   6,   7,   8,   9,  10])
    tensor([ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10])
    

## 张量的索引、分片、合并以及维度调整

    张量作为有序的序列，也是具备数值索引的功能，并且基本索引方法和python原生的列表、numpy中的数组基本一致，当然，所不同的是，pytorch中还定义了一种采用函数来进行索引的方式。
    而作为pytorch中基本数据类型，张量即具备了列表、数组的基本功能，同时还充当着向量、矩阵、数据框等重要的数据结构，因此pytorch中也设置了非常完备的张量合并与变换的操作


```python
import torch
import numpy as np
```

#### 一维张量索引
一维张量的索引过程和python原生对象类型的索引一致,[start:end,step]


```python
t1=torch.arange(1,11)
t1
```




    tensor([ 1,  2,  3,  4,  5,  6,  7,  8,  9, 10])




```python
t1[1:8]
```




    tensor([2, 3, 4, 5, 6, 7, 8])




```python
t1[1:8:2] # 索引其中2-9号元素，左闭右开，每隔2个数取一个
```




    tensor([2, 4, 6, 8])




```python
t1[1: :2] # 从第二个元素开始索引，一直到结尾，并且每隔两个数取一个
```




    tensor([ 2,  4,  6,  8, 10])




```python
t1[:8:2] # 第一个元素到第九个元素（不包含）
```




    tensor([1, 3, 5, 7])



在张量的索引中，step位必须大于0

#### 二维张量
二维张量可以视为两个一维张量组合而成，在实际的索引过程中，需要用逗号进行分割，分别不表示对哪个一维张量进行索引，以及具体的一维张量的索引


```python
t2=torch.arange(1,10).reshape(3,3)
t2
```




    tensor([[1, 2, 3],
            [4, 5, 6],
            [7, 8, 9]])




```python
t2[0,1]
```




    tensor(2)




```python
t2[0,::2]  # 表示索引第一行、每隔两个元素取一个
```




    tensor([1, 3])




```python
t2[::2,::2]
```




    tensor([[1, 3],
            [7, 9]])



#### 三维张量
 即二维张量组成的三维张量


```python
t3=torch.arange(1,28).reshape(3,3,3)
t3
```




    tensor([[[ 1,  2,  3],
             [ 4,  5,  6],
             [ 7,  8,  9]],
    
            [[10, 11, 12],
             [13, 14, 15],
             [16, 17, 18]],
    
            [[19, 20, 21],
             [22, 23, 24],
             [25, 26, 27]]])




```python
t3[1,1,1]
```




    tensor(14)




```python
t3[1,1]
```




    tensor([13, 14, 15])




```python
t3[1]
```




    tensor([[10, 11, 12],
            [13, 14, 15],
            [16, 17, 18]])




```python
t3[1,::2,::2]
```




    tensor([[10, 12],
            [16, 18]])




```python
t3.shape
```




    torch.Size([3, 3, 3])




```python
t2
```




    tensor([[1, 2, 3],
            [4, 5, 6],
            [7, 8, 9]])




```python
indices=torch.tensor([1,2])
indices
```




    tensor([1, 2])




```python
t2.shape
```




    torch.Size([3, 3])




```python
torch.index_select(t2,0,indices) # dim参数为0，代表在shape的第一个维度上索引 
```




    tensor([[4, 5, 6],
            [7, 8, 9]])




```python
torch.index_select(t2,1,indices) # dim参数为1，代表在shape的第二个维度上索引
```




    tensor([[2, 3],
            [5, 6],
            [8, 9]])



## tensor.view()方法
.view方法会返回一个类似视图的结果，通过。view方法还可以改变对象结构，生成一个不同结构，但是是“浅拷贝“


```python
t=torch.arange(6).reshape(2,3)
t
```




    tensor([[0, 1, 2],
            [3, 4, 5]])




```python
te=t.view(3,2)    # 构建一个结构不同但数据一样的“视图”
te
```




    tensor([[0, 1],
            [2, 3],
            [4, 5]])




```python
t[0]=1
t[1][0]=1
```


```python
t
```




    tensor([[1, 1, 1],
            [1, 4, 5]])




```python
te   
```




    tensor([[1, 1],
            [1, 1],
            [4, 5]])




```python
tr=t.view(1,2,3)
tr
```




    tensor([[[1, 1, 1],
             [1, 4, 5]]])



## 张量的分片函数
#### 1. 分块 chunk函数
chunk函数能够按照某维度，对张量进行均匀切分，并返回原张量的视图


```python
t2=torch.arange(12).reshape(4,3)
t2
```




    tensor([[ 0,  1,  2],
            [ 3,  4,  5],
            [ 6,  7,  8],
            [ 9, 10, 11]])




```python
tc=torch.chunk(t2,4,dim=0) # 在第零个维度上进行四等分
tc
```




    (tensor([[0, 1, 2]]),
     tensor([[3, 4, 5]]),
     tensor([[6, 7, 8]]),
     tensor([[ 9, 10, 11]]))




```python
tc[0][0]
```




    tensor([0, 1, 2])




```python
tc[0][0][0]=1
```


```python
tc
```




    (tensor([[1, 1, 2]]),
     tensor([[3, 4, 5]]),
     tensor([[6, 7, 8]]),
     tensor([[ 9, 10, 11]]))




```python
t2.size()  # dim=0 -> 4   dim=1 -> 3
```




    torch.Size([4, 3])




```python
tc=torch.chunk(t2,3,dim=1)
tc
```




    (tensor([[1],
             [3],
             [6],
             [9]]),
     tensor([[ 1],
             [ 4],
             [ 7],
             [10]]),
     tensor([[ 2],
             [ 5],
             [ 8],
             [11]]))




```python
tc=torch.chunk(t2,3,dim=0)  # 当原张量不能均分时，会返回次一级均分的结果
tc
```




    (tensor([[1, 1, 2],
             [3, 4, 5]]),
     tensor([[ 6,  7,  8],
             [ 9, 10, 11]]))




```python
torch.chunk(t2,4) # 默认dim=0
```




    (tensor([[1, 1, 2]]),
     tensor([[3, 4, 5]]),
     tensor([[6, 7, 8]]),
     tensor([[ 9, 10, 11]]))



#### 2.拆分：split函数
split既能进行均分也能进行自定义切分，当然，和chunk函数一样，都是返回视图


```python
t2=torch.arange(12).reshape(4,3)
t2
```




    tensor([[ 0,  1,  2],
            [ 3,  4,  5],
            [ 6,  7,  8],
            [ 9, 10, 11]])




```python
torch.split(t2,2,0) # 第二个参数表示切分的序列  第三个参数表示维度
```




    (tensor([[0, 1, 2],
             [3, 4, 5]]),
     tensor([[ 6,  7,  8],
             [ 9, 10, 11]]))




```python
tc=torch.split(t2,[1,3],0)
tc
```




    (tensor([[0, 1, 2]]),
     tensor([[ 3,  4,  5],
             [ 6,  7,  8],
             [ 9, 10, 11]]))




```python
len(t2)  # 序列的和与len()一致
```




    4




```python
tc[1][0]=torch.tensor([1,2,3])
tc
```




    (tensor([[0, 1, 2]]),
     tensor([[ 1,  2,  3],
             [ 6,  7,  8],
             [ 9, 10, 11]]))



#### 3.拼接
张量的合并操作类似于列表的追加元素，可以拼接、也可以堆叠
- 拼接函数：cat


```python
a=torch.zeros(2,3)
a
```




    tensor([[0., 0., 0.],
            [0., 0., 0.]])




```python
b=torch.ones(2,3)
b
```




    tensor([[1., 1., 1.],
            [1., 1., 1.]])




```python
c=torch.zeros(3,3)
c
```




    tensor([[0., 0., 0.],
            [0., 0., 0.],
            [0., 0., 0.]])




```python
torch.cat([a,b])
```




    tensor([[0., 0., 0.],
            [0., 0., 0.],
            [1., 1., 1.],
            [1., 1., 1.]])




```python
torch.cat([a,b],1) # 按照维度进行拼接，dim默认为0
```




    tensor([[0., 0., 0., 1., 1., 1.],
            [0., 0., 0., 1., 1., 1.]])




```python
torch.car([a,c],1) # 形状不匹配时会报错
```

- 堆叠函数：stack
和拼接不同，堆叠不是将元素拆分重组，而是简单的将各参与堆叠的对象分装到一个更高维度的张量里
堆叠要求张量形状都相同


```python
a
```




    tensor([[0., 0., 0.],
            [0., 0., 0.]])




```python
b
```




    tensor([[1., 1., 1.],
            [1., 1., 1.]])




```python
c
```




    tensor([[0., 0., 0.],
            [0., 0., 0.],
            [0., 0., 0.]])




```python
t3=torch.stack([torch.cat([a,c]),torch.cat([b,c])])
t3
```




    tensor([[[0., 0., 0.],
             [0., 0., 0.],
             [0., 0., 0.],
             [0., 0., 0.],
             [0., 0., 0.]],
    
            [[1., 1., 1.],
             [1., 1., 1.],
             [0., 0., 0.],
             [0., 0., 0.],
             [0., 0., 0.]]])




```python
t3.size()
```




    torch.Size([2, 5, 3])




```python
torch.stack([a,b])
```




    tensor([[[0., 0., 0.],
             [0., 0., 0.]],
    
            [[1., 1., 1.],
             [1., 1., 1.]]])




```python
torch.stack([a,b]).shape
```




    torch.Size([2, 2, 3])



## 张量维度变换
通过reshape方法可以灵活调整张量的形状，在实际使用中往往要使用升维和降维操作
使用squeeze函数进行降维，使用unsqueeze函数进行升维


```python
a=torch.arange(4)
a
```




    tensor([0, 1, 2, 3])




```python
a2=a.reshape(1,4)
a2
```




    tensor([[0, 1, 2, 3]])




```python
torch.squeeze(a2).ndim
```




    1




```python
a2=torch.zeros(1,2,1,2)
a2
```




    tensor([[[[0., 0.]],
    
             [[0., 0.]]]])




```python
torch.unsqueeze(a2,dim=0)  # 在第1个维度索引上升高一个维度
```




    tensor([[[[[0., 0.]],
    
              [[0., 0.]]]]])




```python
torch.unsqueeze(a2,dim=0).shape
```




    torch.Size([1, 1, 2, 1, 2])




```python
torch.unsqueeze(a2,dim=2).shape
```




    torch.Size([1, 2, 1, 1, 2])




```python
torch.unsqueeze(a2,dim=3).shape
```




    torch.Size([1, 2, 1, 1, 2])




```python
torch.unsqueeze(a2,dim=4).shape
```




    torch.Size([1, 2, 1, 2, 1])


