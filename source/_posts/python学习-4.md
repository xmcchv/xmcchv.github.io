---
title: python学习-4
author: xmcchv
date: 2021-07-06 18:13:06
tags: python
---

## 面向对象

面向对象技术简介
- 类(Class): 用来描述具有相同的属性和方法的对象的集合。它定义了该集合中每个对象所共有的属性和方法。对象是类的实例。
- 类变量：类变量在整个实例化的对象中是公用的。类变量定义在类中且在函数体之外。类变量通常不作为实例变量使用。
- 数据成员：类变量或者实例变量, 用于处理类及其实例对象的相关的数据。
- 方法重写：如果从父类继承的方法不能满足子类的需求，可以对其进行改写，这个过程叫方法的覆盖（override），也称为方法的重写。
- 局部变量：定义在方法中的变量，只作用于当前实例的类。
- 实例变量：在类的声明中，属性是用变量来表示的。这种变量就称为实例变量，是在类声明的内部但是在类的其他成员方法之外声明的。
- 继承：即一个派生类（derived class）继承基类（base class）的字段和方法。继承也允许把一个派生类的对象作为一个基类对象对待。
- 实例化：创建一个类的实例，类的具体对象。
- 方法：类中定义的函数。
- 对象：通过类定义的数据结构实例。对象包括两个数据成员（类变量和实例变量）和方法。

#### 1. 创建类
```python 
class ClassName:
   '类的帮助信息'   #类文档字符串
   class_suite  #类体
```

以下是一个简单的 Python 类的例子:
```python
class Employee:
   '所有员工的基类'
   empCount = 0
 
   def __init__(self, name, salary):
      self.name = name
      self.salary = salary
      Employee.empCount += 1
   
   def displayCount(self):
     print "Total Employee %d" % Employee.empCount
 
   def displayEmployee(self):
      print "Name : ", self.name,  ", Salary: ", self.salary
```
- empCount 变量是一个类变量，它的值将在这个类的所有实例之间共享。你可以在内部类或外部类使用 Employee.empCount 访问。与Java不同，Java类的实例中的变量都是不共享的，除非声明为static。

- 第一种方法__init__()方法是一种特殊的方法，被称为类的构造函数或初始化方法，当创建了这个类的实例时就会调用该方法

- self 代表类的实例，self 在定义类的方法时是必须有的，虽然在调用时不必传入相应的参数。例如t.prt(),self在python语句中传入的就是t。

#### 2. 类的继承
面向对象的编程带来的主要好处之一是代码的重用，实现这种重用的方法之一是通过继承机制。
通过继承创建的新类称为子类或派生类，被继承的类称为基类、父类或超类。

继承语法
```python
class 派生类名(基类名)
    ...
```
在python中继承中的一些特点：
- 1、如果在子类中需要父类的构造方法就需要显式的调用父类的构造方法，或者不重写父类的构造方法。详细说明可查看： python 子类继承父类构造函数说明。
- 2、在调用基类的方法时，需要加上基类的类名前缀，且需要带上 self 参数变量。区别在于类中调用普通函数时并不需要带上 self 参数
- 3、Python 总是首先查找对应类型的方法，如果它不能在派生类中找到对应的方法，它才开始到基类中逐个查找。（先在本类中查找调用的方法，找不到才去基类中找）。
如果在继承元组中列了一个以上的类，那么它就被称作"多重继承" 。

语法：
派生类的声明，与他们的父类类似，继承的基类列表跟在类名之后，如下所示：
```python
class SubClassName (ParentClass1[, ParentClass2, ...]):
    ...
```
实例:
```python
class Parent:        # 定义父类
   parentAttr = 100
   def __init__(self):
      print "调用父类构造函数"
 
   def parentMethod(self):
      print '调用父类方法'
 
   def setAttr(self, attr):
      Parent.parentAttr = attr
 
   def getAttr(self):
      print "父类属性 :", Parent.parentAttr
 
class Child(Parent): # 定义子类
   def __init__(self):
      print "调用子类构造方法"
 
   def childMethod(self):
      print '调用子类方法'
 
c = Child()          # 实例化子类
c.childMethod()      # 调用子类的方法
c.parentMethod()     # 调用父类方法
c.setAttr(200)       # 再次调用父类的方法 - 设置属性值
c.getAttr()          # 再次调用父类的方法 - 获取属性值
```

#### 3. 方法重写
如果你的父类方法的功能不能满足你的需求，你可以在子类重写你父类的方法：
```python
class Parent:        # 定义父类
   def myMethod(self):
      print '调用父类方法'
 
class Child(Parent): # 定义子类
   def myMethod(self):
      print '调用子类方法'
 
c = Child()          # 子类实例
c.myMethod()         # 子类调用重写方法
```

#### 4. 类属性与方法
- 类的私有属性
__private_attrs：两个下划线开头，声明该属性为私有，不能在类的外部被使用或直接访问。在类内部的方法中使用时 self.__private_attrs。

- 类的方法
在类的内部，使用 def 关键字可以为类定义一个方法，与一般函数定义不同，类方法必须包含参数 self,且为第一个参数

- 类的私有方法
__private_method：两个下划线开头，声明该方法为私有方法，不能在类的外部调用。在类的内部调用 self.__private_methods

实例
```python
class JustCounter:
    __secretCount = 0  # 私有变量
    publicCount = 0    # 公开变量
 
    def count(self):
        self.__secretCount += 1
        self.publicCount += 1
        print self.__secretCount
 
counter = JustCounter()
counter.count()
counter.count()
print counter.publicCount
print counter.__secretCount  # 报错，实例不能访问私有变量
```

单下划线、双下划线、头尾双下划线说明：
- _foo__: 定义的是特殊方法，一般是系统定义名字 ，类似 __init__() 之类的。

- _foo: 以单下划线开头的表示的是 protected 类型的变量，即保护类型只能允许其本身与子类进行访问，不能用于 from module import *

- __foo: 双下划线的表示的是私有类型(private)的变量, 只能是允许这个类本身进行访问了。


#### 5. 运算符重载
Python同样支持运算符重载。
```python
class Vector:
   def __init__(self, a, b):
      self.a = a
      self.b = b
 
   def __str__(self):
      return 'Vector (%d, %d)' % (self.a, self.b)
   
   def __add__(self,other):
      return Vector(self.a + other.a, self.b + other.b)
 
v1 = Vector(2,10)
v2 = Vector(5,-2)
print v1 + v2
# 以上代码执行结果如下所示:
# Vector(7,8)
```



## 正则表达式

#### 1. re.match函数
re.match 尝试从字符串的起始位置匹配一个模式，如果不是起始位置匹配成功的话，match()就返回none。

函数语法：
`re.match(pattern, string, flags=0)`

#### 2. re.search方法
re.search 扫描整个字符串并返回第一个成功的匹配。

函数语法：
`re.search(pattern, string, flags=0)`

###### re.match与re.search的区别
re.match只匹配字符串的开始，如果字符串开始不符合正则表达式，则匹配失败，函数返回None；而re.search匹配整个字符串，直到找到一个匹配。

#### 3. 检索和替换
Python 的 re 模块提供了re.sub用于替换字符串中的匹配项。

语法：
`re.sub(pattern, repl, string, count=0, flags=0)`
参数：
- pattern : 正则中的模式字符串。
- repl : 替换的字符串，也可为一个函数。
- string : 要被查找替换的原始字符串。
- count : 模式匹配后替换的最大次数，默认 0 表示替换所有的匹配。

#### 4. re.compile 函数
compile 函数用于编译正则表达式，生成一个正则表达式（ Pattern ）对象，供 match() 和 search() 这两个函数使用。

语法格式为：
`re.compile(pattern[, flags])`
参数：
- `pattern `: 一个字符串形式的正则表达式
- `flags `: 可选，表示匹配模式，比如忽略大小写，多行模式等，具体参数为：
    - re.I 忽略大小写
    - re.L 表示特殊字符集 \w, \W, \b, \B, \s, \S 依赖于当前环境
    - re.M 多行模式
    - re.S 即为 `.` 并且包括换行符在内的任意字符（`. `不包括换行符）
    - re.U 表示特殊字符集 \w, \W, \b, \B, \d, \D, \s, \S 依赖于 Unicode 字符属性数据库
    - re.X 为了增加可读性，忽略空格和` # `后面的注释


#### 5. re.finditer
和 findall 类似，在字符串中找到正则表达式所匹配的所有子串，并把它们作为一个迭代器返回。
`re.finditer(pattern, string, flags=0)`


#### 6. re.split
split 方法按照能够匹配的子串将字符串分割后返回列表，它的使用形式如下：
`re.split(pattern, string[, maxsplit=0, flags=0])`

#### 7. 正则表达式对象
1. re.RegexObject
re.compile() 返回 RegexObject 对象。

2. re.MatchObject
    - group() 返回被 RE 匹配的字符串。
    - start() 返回匹配开始的位置
    - end() 返回匹配结束的位置
    - span() 返回一个元组包含匹配 (开始,结束) 的位置

#### 8. 正则表达式模式
模式字符串使用特殊的语法来表示一个正则表达式：
字母和数字表示他们自身。一个正则表达式模式中的字母和数字匹配同样的字符串。
多数字母和数字前加一个反斜杠时会拥有不同的含义。
标点符号只有被转义时才匹配自身，否则它们表示特殊的含义。
反斜杠本身需要使用反斜杠转义。
由于正则表达式通常都包含反斜杠，所以你最好使用原始字符串来表示它们。模式元素(如 r'\t'，等价于 '\\t')匹配相应的特殊字符。
下表列出了正则表达式模式语法中的特殊元素。如果你使用模式的同时提供了可选的标志参数，某些模式元素的含义会改变。
模式	|描述
:-|:-
^	|匹配字符串的开头
$	|匹配字符串的末尾。
.	|匹配任意字符，除了换行符，当re.DOTALL标记被指定时，则可以匹配包括换行符的任意字符。
[...]	|用来表示一组字符,单独列出：[amk] 匹配 'a'，'m'或'k'
[^...]|	不在[]中的字符：[^abc] 匹配除了a,b,c之外的字符。
re*	|匹配0个或多个的表达式。
re+	|匹配1个或多个的表达式。
re?	|匹配0个或1个由前面的正则表达式定义的片段，非贪婪方式
re{ n}	|精确匹配 n 个前面表达式。例如， o{2} 不能匹配 "Bob" 中的 "o"，但是能匹配 "food" 中的两个 o。
re{ n,}	|匹配 n 个前面表达式。例如， o{2,} 不能匹配"Bob"中的"o"，但能匹配 "foooood"中的所有 o。"o{1,}" 等价于 "o+"。"o{0,}" 则等价于 "o*"。
re{ n, m}	|匹配 n 到 m 次由前面的正则表达式定义的片段，贪婪方式
a| b	|匹配a或b
(re)|	对正则表达式分组并记住匹配的文本
(?imx)	|正则表达式包含三种可选标志：i, m, 或 x 。只影响括号中的区域。
(?-imx)	|正则表达式关闭 i, m, 或 x 可选标志。只影响括号中的区域。
(?: re)	|类似 (...), 但是不表示一个组
(?imx: re)|	在括号中使用i, m, 或 x 可选标志
(?-imx: re)	|在括号中不使用i, m, 或 x 可选标志
(?#...)	|注释.
(?= re)	|前向肯定界定符。如果所含正则表达式，以 ... 表示，在当前位置成功匹配时成功，否则失败。但一旦所含表达式已经尝试，匹配引擎根本没有提高；模式的剩余部分还要尝试界定符的右边。
(?! re)	|前向否定界定符。与肯定界定符相反；当所含表达式不能在字符串当前位置匹配时成功
(?> re)|	匹配的独立模式，省去回溯。
\w	|匹配字母数字及下划线
\W|	匹配非字母数字及下划线
\s	|匹配任意空白字符，等价于 [ \t\n\r\f]。
\S	|匹配任意非空字符
\d	|匹配任意数字，等价于 [0-9].
\D	|匹配任意非数字
\A	|匹配字符串开始
\Z	|匹配字符串结束，如果是存在换行，只匹配到换行前的结束字符串。
\z	|匹配字符串结束
\G	|匹配最后匹配完成的位置。
\b	|匹配一个单词边界，也就是指单词和空格间的位置。例如， 'er\b' 可以匹配"never" 中的 'er'，但不能匹配 "verb" 中的 'er'。
\B	|匹配非单词边界。'er\B' 能匹配 "verb" 中的 'er'，但不能匹配 "never" 中的 'er'。
\n, \t, 等.|	匹配一个换行符。匹配一个制表符。等
\1...\9|	匹配第n个分组的内容。
\10	|匹配第n个分组的内容，如果它经匹配。否则指的是八进制字符码的表达式。


##### '(?P...)' 分组匹配

例：身份证 1102231990xxxxxxxx
```python
import re
s = '1102231990xxxxxxxx'
res = re.search('(?P<province>\d{3})(?P<city>\d{3})(?P<born_year>\d{4})',s)
print(res.groupdict())
```
此分组取出结果为：
`{'province': '110', 'city': '223', 'born_year': '1990'}`
直接将匹配结果直接转为字典模式，方便使用。





























