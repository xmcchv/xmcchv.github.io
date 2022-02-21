<<<<<<< HEAD
---
title: Python学习-1
author: xmcchv
date: 2021-07-03 17:10:54
tags: python
---
## 基础知识
1. 一般格式

```python 
# coding:utf-8     
# 一个空行
import os
# 两个空行
print(os.getcwd())
print('hello,world!')
list = ["a","b","c"]
print(list)
# 一个空行
```
2. 注释

```python
# 单行注释
'''
这是多行注释，使用单引号。
这是多行注释，使用单引号。
这是多行注释，使用单引号。
'''
"""
这是多行注释，使用双引号。
这是多行注释，使用双引号。
这是多行注释，使用双引号。
"""
```

## 数据类型
#### Numbers（数字）

```python 
counter = 100 # 赋值整型变量
miles = 1000.0 # 浮点型
name = "xmcchv" # 字符串
a, b, c = 1, 2, "john"
```

#### String（字符串）
python的字串列表有2种取值顺序:
从左到右索引默认0开始的，最大范围是字符串长度少1
从右到左索引默认-1开始的，最大范围是字符串开头
![python-string-slice](/images/list_slicing1_new1.png "python-string-slice")

#### List（列表）

List（列表） 是 Python 中使用最频繁的数据类型。

列表可以完成大多数集合类的数据结构实现。它支持字符，数字，字符串甚至可以包含列表（即嵌套）。

列表用` [ ] `标识，是 python 最通用的复合数据类型。

列表中值的切割也可以用到变量` [头下标:尾下标] `，就可以截取相应的列表，从左到右索引默认 0 开始，从右到左索引默认 -1 开始，下标可以为空表示取到头或尾。
![list_slicing](/images/list_slicing1_new1.png "list_slicing")
加号` + `是列表连接运算符，星号` * `是重复操作。如下实例：
```python
list = [ 'runoob', 786 , 2.23, 'john', 70.2 ]
tinylist = [123, 'john']
 
print list               # 输出完整列表
print list[0]            # 输出列表的第一个元素
print list[1:3]          # 输出第二个至第三个元素 
print list[2:]           # 输出从第三个开始至列表末尾的所有元素
print tinylist * 2       # 输出列表两次
print list + tinylist    # 打印组合的列表

```

#### Tuple（元组）

元组是另一个数据类型，类似于 List（列表）。

元组用` () `标识。内部元素用逗号隔开。但是元组不能二次赋值，相当于只读列表。


#### Dictionary（字典）

字典(dictionary)是除列表以外python之中最灵活的内置数据结构类型。列表是有序的对象集合，字典是无序的对象集合。

两者之间的区别在于：字典当中的元素是通过键来存取的，而不是通过偏移存取。

字典用`{ }`标识。字典由索引(key)和它对应的值value组成。

```python
dict = {}
dict['one'] = "This is one"
dict[2] = "This is two"
 
tinydict = {'name': 'runoob','code':6734, 'dept': 'sales'}

print dict['one']          # 输出键为'one' 的值
print dict[2]              # 输出键为 2 的值
print tinydict             # 输出完整的字典
print tinydict.keys()      # 输出所有键
print tinydict.values()    # 输出所有值
```
输出结果如下：
```bash
This is one
This is two
{'dept': 'sales', 'code': 6734, 'name': 'runoob'}
['dept', 'code', 'name']
['sales', 6734, 'runoob']
```


## 算数运算符

1. 用`**`表示幂 - 返回x的y次幂
2. 用`//`取整除 - 返回商的整数部分（向下取整）
   实例：
   ```python
   >>> 9//2
   4
   >>> -9//2
   -5
   ```

3. 用`**=`表示幂赋值运算符`	c **= a `等效于` c = c ** a`
4. 用`//=`表示取整除赋值运算符`	c //= a `等效于` c = c // a`

## 位运算
按位运算符是把数字看作二进制来进行计算的。
下表中变量 a 为 60，b 为 13，二进制格式如下：
```python
a = 0011 1100
b = 0000 1101

a&b = 0000 1100
a|b = 0011 1101
a^b = 0011 0001
~a  = 1100 0011
```
![位运算](/images/weiyunsuan.png "位运算")

## 成员运算符
运算符|	描述	|实例
:-|:-|:-
in	|如果在指定的序列中找到值返回 True，否则返回 False。	|x 在 y 序列中 , 如果 x 在 y 序列中返回 True。
not in|	如果在指定的序列中没有找到值返回 True，否则返回 False。|	x 不在 y 序列中 , 如果 x 不在 y 序列中返回 True。


## 身份运算符
身份运算符用于比较两个对象的存储单元

运算符|	描述	|实例
:-|:-|:-
is|	is 是判断两个标识符是不是引用自一个对象|	x is y, 类似 id(x) == id(y) , 如果引用的是同一个对象则返回 True，否则返回 False
is not|	is not 是判断两个标识符是不是引用自不同对象|	x is not y ， 类似 id(a) != id(b)。如果引用的不是同一个对象则返回结果 True，否则返回 False。

```
is 与 == 区别：
is 用于判断两个变量引用对象是否为同一个(同一块内存空间)
== 用于判断引用变量的值是否相等。
```



文章都来自菜鸟教程，这里我挑了一下做了下整理，都是我不会的QAQ



=======
---
title: Python学习-1
author: xmcchv
date: 2021-07-03 17:10:54
tags: python
---
## 基础知识
1. 一般格式

```python 
# coding:utf-8     
# 一个空行
import os
# 两个空行
print(os.getcwd())
print('hello,world!')
list = ["a","b","c"]
print(list)
# 一个空行
```
2. 注释

```python
# 单行注释
'''
这是多行注释，使用单引号。
这是多行注释，使用单引号。
这是多行注释，使用单引号。
'''
"""
这是多行注释，使用双引号。
这是多行注释，使用双引号。
这是多行注释，使用双引号。
"""
```

## 数据类型
#### Numbers（数字）

```python 
counter = 100 # 赋值整型变量
miles = 1000.0 # 浮点型
name = "xmcchv" # 字符串
a, b, c = 1, 2, "john"
```

#### String（字符串）
python的字串列表有2种取值顺序:
从左到右索引默认0开始的，最大范围是字符串长度少1
从右到左索引默认-1开始的，最大范围是字符串开头
![python-string-slice](/images/list_slicing1_new1.png "python-string-slice")

#### List（列表）

List（列表） 是 Python 中使用最频繁的数据类型。

列表可以完成大多数集合类的数据结构实现。它支持字符，数字，字符串甚至可以包含列表（即嵌套）。

列表用` [ ] `标识，是 python 最通用的复合数据类型。

列表中值的切割也可以用到变量` [头下标:尾下标] `，就可以截取相应的列表，从左到右索引默认 0 开始，从右到左索引默认 -1 开始，下标可以为空表示取到头或尾。
![list_slicing](/images/list_slicing1_new1.png "list_slicing")
加号` + `是列表连接运算符，星号` * `是重复操作。如下实例：
```python
list = [ 'runoob', 786 , 2.23, 'john', 70.2 ]
tinylist = [123, 'john']
 
print list               # 输出完整列表
print list[0]            # 输出列表的第一个元素
print list[1:3]          # 输出第二个至第三个元素 
print list[2:]           # 输出从第三个开始至列表末尾的所有元素
print tinylist * 2       # 输出列表两次
print list + tinylist    # 打印组合的列表

```

#### Tuple（元组）

元组是另一个数据类型，类似于 List（列表）。

元组用` () `标识。内部元素用逗号隔开。但是元组不能二次赋值，相当于只读列表。


#### Dictionary（字典）

字典(dictionary)是除列表以外python之中最灵活的内置数据结构类型。列表是有序的对象集合，字典是无序的对象集合。

两者之间的区别在于：字典当中的元素是通过键来存取的，而不是通过偏移存取。

字典用`{ }`标识。字典由索引(key)和它对应的值value组成。

```python
dict = {}
dict['one'] = "This is one"
dict[2] = "This is two"
 
tinydict = {'name': 'runoob','code':6734, 'dept': 'sales'}

print dict['one']          # 输出键为'one' 的值
print dict[2]              # 输出键为 2 的值
print tinydict             # 输出完整的字典
print tinydict.keys()      # 输出所有键
print tinydict.values()    # 输出所有值
```
输出结果如下：
```bash
This is one
This is two
{'dept': 'sales', 'code': 6734, 'name': 'runoob'}
['dept', 'code', 'name']
['sales', 6734, 'runoob']
```


## 算数运算符

1. 用`**`表示幂 - 返回x的y次幂
2. 用`//`取整除 - 返回商的整数部分（向下取整）
   实例：
   ```python
   >>> 9//2
   4
   >>> -9//2
   -5
   ```

3. 用`**=`表示幂赋值运算符`	c **= a `等效于` c = c ** a`
4. 用`//=`表示取整除赋值运算符`	c //= a `等效于` c = c // a`

## 位运算
按位运算符是把数字看作二进制来进行计算的。
下表中变量 a 为 60，b 为 13，二进制格式如下：
```python
a = 0011 1100
b = 0000 1101

a&b = 0000 1100
a|b = 0011 1101
a^b = 0011 0001
~a  = 1100 0011
```
![位运算](/images/weiyunsuan.png "位运算")

## 成员运算符
运算符|	描述	|实例
:-|:-|:-
in	|如果在指定的序列中找到值返回 True，否则返回 False。	|x 在 y 序列中 , 如果 x 在 y 序列中返回 True。
not in|	如果在指定的序列中没有找到值返回 True，否则返回 False。|	x 不在 y 序列中 , 如果 x 不在 y 序列中返回 True。


## 身份运算符
身份运算符用于比较两个对象的存储单元

运算符|	描述	|实例
:-|:-|:-
is|	is 是判断两个标识符是不是引用自一个对象|	x is y, 类似 id(x) == id(y) , 如果引用的是同一个对象则返回 True，否则返回 False
is not|	is not 是判断两个标识符是不是引用自不同对象|	x is not y ， 类似 id(a) != id(b)。如果引用的不是同一个对象则返回结果 True，否则返回 False。

```
is 与 == 区别：
is 用于判断两个变量引用对象是否为同一个(同一块内存空间)
== 用于判断引用变量的值是否相等。
```



文章都来自菜鸟教程，这里我挑了一下做了下整理，都是我不会的QAQ



>>>>>>> 285b814601d5399a58128ce8798b99dda8ed6d59
