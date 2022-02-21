<<<<<<< HEAD
---
title: python学习-2
author: xmcchv
date: 2021-07-03 22:28:28
tags: python
---
## 日期和时间
```python
import time  # 引入time模块

# 时间戳
ticks = time.time()
print "当前时间戳为:", ticks
当前时间戳为: 1625322855.8431683

# 时间
localtime = time.localtime(time.time())
print "本地时间为 :", localtime
# time.struct_time(tm_year=2021, tm_mon=7, tm_mday=3, tm_hour=22, 
# tm_min=33, tm_sec=47, tm_wday=5, tm_yday=184, tm_isdst=0)

# 格式化时间
# 格式化成2021-07-03 22:44:36形式
print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())) 
 
# 格式化成Sat Jul 03 22:44:36 2021形式
print(time.strftime("%a %b %d %H:%M:%S %Y", time.localtime()))
  
# 将格式字符串转换为时间戳
a = "Sat Jul 03 22:44:36 2021"
print(time.mktime(time.strptime(a,"%a %b %d %H:%M:%S %Y")))

'''
2021-07-03 22:45:30
Sat Jul 03 22:45:30 2021
1625323476.0
'''
```


## 日历
```python
import calendar
cal = calendar.month(2021, 7)
print("以下输出2021年7月份的日历:\n",cal)

'''
以下输出2021年7月份的日历:
      July 2021
Mo Tu We Th Fr Sa Su
          1  2  3  4
 5  6  7  8  9 10 11
12 13 14 15 16 17 18
19 20 21 22 23 24 25
26 27 28 29 30 31
'''
```

## 函数
你可以定义一个由自己想要功能的函数，以下是简单的规则：

- 函数代码块以` def `关键词开头，后接函数标识符名称和圆括号`()`。
- 任何传入参数和自变量必须放在圆括号中间。圆括号之间可以用于定义参数。
- 函数的第一行语句可以选择性地使用文档字符串—用于存放函数说明。
- 函数内容以冒号起始，并且缩进。
- `return [表达式] `结束函数，选择性地返回一个值给调用方。不带表达式的return相当于返回 None。

```python 
def  functionname( parameters ):
   "函数_文档字符串"
   function_suite
   return [expression]
```
#### 默认参数
调用函数时，默认参数的值如果没有传入，则被认为是默认值。下例会打印默认的age，如果age没有被传入：
```python
#可写函数说明
def printinfo( name, age = 35 ):
   "打印任何传入的字符串"
   print "Name: ", name
   print "Age ", age
   return
 
#调用printinfo函数
printinfo( age=50, name="miki" )
printinfo( name="miki" )

'''
Name:  miki
Age  50
Name:  miki
Age  35
'''
```


#### 不定长函数
需要一个函数能处理比当初声明时更多的参数。这些参数叫做不定长参数，参数在函数声明时不会命名。
```python
# 可写函数说明
def printinfo( arg1, *vartuple ):
   "打印任何传入的参数"
   print "输出: "
   print arg1
   for var in vartuple:
      print var
   return
 
# 调用printinfo 函数
printinfo( 10 )
printinfo( 70, 60, 50 )
'''
输出:
10
输出:
70
60
50
'''
```




=======
---
title: python学习-2
author: xmcchv
date: 2021-07-03 22:28:28
tags: python
---
## 日期和时间
```python
import time  # 引入time模块

# 时间戳
ticks = time.time()
print "当前时间戳为:", ticks
当前时间戳为: 1625322855.8431683

# 时间
localtime = time.localtime(time.time())
print "本地时间为 :", localtime
# time.struct_time(tm_year=2021, tm_mon=7, tm_mday=3, tm_hour=22, 
# tm_min=33, tm_sec=47, tm_wday=5, tm_yday=184, tm_isdst=0)

# 格式化时间
# 格式化成2021-07-03 22:44:36形式
print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())) 
 
# 格式化成Sat Jul 03 22:44:36 2021形式
print(time.strftime("%a %b %d %H:%M:%S %Y", time.localtime()))
  
# 将格式字符串转换为时间戳
a = "Sat Jul 03 22:44:36 2021"
print(time.mktime(time.strptime(a,"%a %b %d %H:%M:%S %Y")))

'''
2021-07-03 22:45:30
Sat Jul 03 22:45:30 2021
1625323476.0
'''
```


## 日历
```python
import calendar
cal = calendar.month(2021, 7)
print("以下输出2021年7月份的日历:\n",cal)

'''
以下输出2021年7月份的日历:
      July 2021
Mo Tu We Th Fr Sa Su
          1  2  3  4
 5  6  7  8  9 10 11
12 13 14 15 16 17 18
19 20 21 22 23 24 25
26 27 28 29 30 31
'''
```

## 函数
你可以定义一个由自己想要功能的函数，以下是简单的规则：

- 函数代码块以` def `关键词开头，后接函数标识符名称和圆括号`()`。
- 任何传入参数和自变量必须放在圆括号中间。圆括号之间可以用于定义参数。
- 函数的第一行语句可以选择性地使用文档字符串—用于存放函数说明。
- 函数内容以冒号起始，并且缩进。
- `return [表达式] `结束函数，选择性地返回一个值给调用方。不带表达式的return相当于返回 None。

```python 
def  functionname( parameters ):
   "函数_文档字符串"
   function_suite
   return [expression]
```
#### 默认参数
调用函数时，默认参数的值如果没有传入，则被认为是默认值。下例会打印默认的age，如果age没有被传入：
```python
#可写函数说明
def printinfo( name, age = 35 ):
   "打印任何传入的字符串"
   print "Name: ", name
   print "Age ", age
   return
 
#调用printinfo函数
printinfo( age=50, name="miki" )
printinfo( name="miki" )

'''
Name:  miki
Age  50
Name:  miki
Age  35
'''
```


#### 不定长函数
需要一个函数能处理比当初声明时更多的参数。这些参数叫做不定长参数，参数在函数声明时不会命名。
```python
# 可写函数说明
def printinfo( arg1, *vartuple ):
   "打印任何传入的参数"
   print "输出: "
   print arg1
   for var in vartuple:
      print var
   return
 
# 调用printinfo 函数
printinfo( 10 )
printinfo( 70, 60, 50 )
'''
输出:
10
输出:
70
60
50
'''
```




>>>>>>> 285b814601d5399a58128ce8798b99dda8ed6d59
