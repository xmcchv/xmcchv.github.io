<<<<<<< HEAD
---
title: python学习-3
author: xmcchv
date: 2021-07-05 17:29:56
tags: python
---

## 列表
Python中列表是可变的，这是它区别于字符串和元组的最重要的特点，一句话概括即：列表可以修改，而字符串和元组不能。

以下是 Python 中列表的方法：


方法|描述
:-|:-
list.append(x)|	把一个元素添加到列表的结尾，相当于 a[len(a):] = [x]。
list.extend(L)|	通过添加指定列表的所有元素来扩充列表，相当于 a[len(a):] = L。
list.insert(i, x)	|在指定位置插入一个元素。第一个参数是准备插入到其前面的那个元素的索引，例如 a.insert(0, x) 会插入到整个列表之前，而 a.insert(len(a), x) 相当于 a.append(x) 。
list.remove(x)|	删除列表中值为 x 的第一个元素。如果没有这样的元素，就会返回一个错误。
list.pop([i])|	从列表的指定位置移除元素，并将其返回。如果没有指定索引，a.pop()返回最后一个元素。元素随即从列表中被移除。（方法中 i 两边的方括号表示这个参数是可选的，而不是要求你输入一对方括号，你会经常在 Python 库参考手册中遇到这样的标记。）
list.clear()|	移除列表中的所有项，等于del a[:]。
list.index(x)|	返回列表中第一个值为 x 的元素的索引。如果没有匹配的元素就会返回一个错误。
list.count(x)	|返回 x 在列表中出现的次数。
list.sort()|	对列表中的元素进行排序。
list.reverse()	|倒排列表中的元素。
list.copy()	|返回列表的浅复制，等于a[:]。


## 列表推导式
列表推导式提供了从序列创建列表的简单途径。通常应用程序将一些操作应用于某个序列的每个元素，用其获得的结果作为生成新列表的元素，或者根据确定的判定条件创建子序列。

每个列表推导式都在 for 之后跟一个表达式，然后有零到多个 for 或 if 子句。返回结果是一个根据表达从其后的 for 和 if 上下文环境中生成出来的列表。如果希望表达式推导出一个元组，就必须使用括号。
```python
>>> vec = [2, 4, 6]
>>> [3*x for x in vec]
# [6, 12, 18]
>>> [[x, x**2] for x in vec]
# [[2, 4], [4, 16], [6, 36]]

```

## 嵌套列表解析
Python的列表还可以嵌套。

以下实例展示了3X4的矩阵列表：

```python
>>> matrix = [
     [1, 2, 3, 4],
     [5, 6, 7, 8],
     [9, 10, 11, 12],]
```
以下实例将3X4的矩阵列表转换为4X3列表：
```python
>>> [[row[i] for row in matrix] for i in range(4)]
[[1, 5, 9], [2, 6, 10], [3, 7, 11], [4, 8, 12]]
```
以下实例也可以使用以下方法来实现：
```python
>>> transposed = []
>>> for i in range(4):
        transposed.append([row[i] for row in matrix])

>>> transposed
[[1, 5, 9], [2, 6, 10], [3, 7, 11], [4, 8, 12]]
```
另外一种实现方法：
```python
>>> transposed = []
>>> for i in range(4):
     # the following 3 lines implement the nested listcomp
     transposed_row = []
     for row in matrix:
         transposed_row.append(row[i])
     transposed.append(transposed_row)

>>> transposed
[[1, 5, 9], [2, 6, 10], [3, 7, 11], [4, 8, 12]]
```

## 文件I/O
### 读取键盘输入
Python提供了两个内置函数从标准输入读入一行文本，默认的标准输入是键盘。如下：

- raw_input
- input

#### 1. raw_input函数
raw_input([prompt]) 函数从标准输入读取一个行，并返回一个字符串（去掉结尾的换行符）：

```python
str = raw_input("请输入：")
print "你输入的内容是: ", str
# 这将提示你输入任意字符串，然后在屏幕上显示相同的字符串。当我输入"Hello Python！"，它的输出如下：
# 请输入：Hello Python！
# 你输入的内容是:  Hello Python！
```
#### 2. input函数
input([prompt]) 函数和 raw_input([prompt]) 函数基本类似，但是 input 可以接收一个Python表达式作为输入，并将运算结果返回。
```python
str = input("请输入：")
print "你输入的内容是: ", str
# 这会产生如下的对应着输入的结果：
# 请输入：[x*5 for x in range(2,10,2)]
# 你输入的内容是:  [10, 20, 30, 40]
```

## 打开和关闭文件
现在，您已经可以向标准输入和输出进行读写。现在，来看看怎么读写实际的数据文件。

Python 提供了必要的函数和方法进行默认情况下的文件基本操作。你可以用 file 对象做大部分的文件操作。

#### 1. open 函数
你必须先用Python内置的open()函数打开一个文件，创建一个file对象，相关的方法才可以调用它进行读写。

语法：
`file object = open(file_name [, access_mode][, buffering])`
各个参数的细节如下：

- file_name：file_name变量是一个包含了你要访问的文件名称的字符串值。
- access_mode：access_mode决定了打开文件的模式：只读，写入，追加等。所有可取值见如下的完全列表。这个参数是非强制的，默认文件访问模式为只读(r)。
- buffering:如果buffering的值被设为0，就不会有寄存。如果buffering的值取1，访问文件时会寄存行。如果将buffering的值设为大于1的整数，表明了这就是的寄存区的缓冲大小。如果取负值，寄存区的缓冲大小则为系统默认。

```python
fo = open("foo.txt", "w")
print "文件名: ", fo.name
print "是否已关闭 : ", fo.closed
print "访问模式 : ", fo.mode
print "末尾是否强制加空格 : ", fo.softspace
'''
文件名:  foo.txt
是否已关闭 :  False
访问模式 :  w
末尾是否强制加空格 :  0
'''
```

#### 2. close()方法
File 对象的 close（）方法刷新缓冲区里任何还没写入的信息，并关闭该文件，这之后便不能再进行写入。

当一个文件对象的引用被重新指定给另一个文件时，Python 会关闭之前的文件。用 close（）方法关闭文件是一个很好的习惯。

#### 3. write()方法
write()方法可将任何字符串写入一个打开的文件。需要重点注意的是，Python字符串可以是二进制数据，而不是仅仅是文字。

write()方法不会在字符串的结尾添加换行符('\n')：

语法：
`fileObject.write(string)`
在这里，被传递的参数是要写入到已打开文件的内容。

例子：
```python
# 打开一个文件
fo = open("foo.txt", "w")
fo.write( "www.runoob.com!\nVery good site!\n")
 
# 关闭打开的文件
fo.close()
```

#### 4. read()方法
read（）方法从一个打开的文件中读取一个字符串。需要重点注意的是，Python字符串可以是二进制数据，而不是仅仅是文字。

语法：
`fileObject.read([count])`
在这里，被传递的参数是要从已打开文件中读取的字节计数。该方法从文件的开头开始读入，如果没有传入count，它会尝试尽可能多地读取更多的内容，很可能是直到文件的末尾。

```python
# 打开一个文件
fo = open("foo.txt", "r+")
str = fo.read(10)
print "读取的字符串是 : ", str
# 关闭打开的文件
fo.close()
```
以上实例输出结果：
`读取的字符串是 :  www.runoob`


## 重命名和删除文件
Python的os模块提供了帮你执行文件处理操作的方法，比如重命名和删除文件。

要使用这个模块，你必须先导入它，然后才可以调用相关的各种功能。

#### 1. rename() 方法
rename() 方法需要两个参数，当前的文件名和新文件名。

语法：
`os.rename(current_file_name, new_file_name)`

#### 2. remove()方法
你可以用remove()方法删除文件，需要提供要删除的文件名作为参数。

语法：
`os.remove(file_name)`


## 目录

#### 1. mkdir()
可以使用os模块的mkdir()方法在当前目录下创建新的目录们。你需要提供一个包含了要创建的目录名称的参数。

语法：
`os.mkdir("newdir")`

#### 2. chdir()
可以用chdir()方法来改变当前的目录。chdir()方法需要的一个参数是你想设成当前目录的目录名称。

语法：
`os.chdir("newdir")`

#### 3. getcwd()方法：
getcwd()方法显示当前的工作目录。

语法：
`os.getcwd()`

#### 4. rmdir()方法
rmdir()方法删除目录，目录名称以参数传递。
在删除这个目录之前，它的所有内容应该先被清除。

语法：
`os.rmdir('dirname')`



















=======
---
title: python学习-3
author: xmcchv
date: 2021-07-05 17:29:56
tags: python
---

## 列表
Python中列表是可变的，这是它区别于字符串和元组的最重要的特点，一句话概括即：列表可以修改，而字符串和元组不能。

以下是 Python 中列表的方法：


方法|描述
:-|:-
list.append(x)|	把一个元素添加到列表的结尾，相当于 a[len(a):] = [x]。
list.extend(L)|	通过添加指定列表的所有元素来扩充列表，相当于 a[len(a):] = L。
list.insert(i, x)	|在指定位置插入一个元素。第一个参数是准备插入到其前面的那个元素的索引，例如 a.insert(0, x) 会插入到整个列表之前，而 a.insert(len(a), x) 相当于 a.append(x) 。
list.remove(x)|	删除列表中值为 x 的第一个元素。如果没有这样的元素，就会返回一个错误。
list.pop([i])|	从列表的指定位置移除元素，并将其返回。如果没有指定索引，a.pop()返回最后一个元素。元素随即从列表中被移除。（方法中 i 两边的方括号表示这个参数是可选的，而不是要求你输入一对方括号，你会经常在 Python 库参考手册中遇到这样的标记。）
list.clear()|	移除列表中的所有项，等于del a[:]。
list.index(x)|	返回列表中第一个值为 x 的元素的索引。如果没有匹配的元素就会返回一个错误。
list.count(x)	|返回 x 在列表中出现的次数。
list.sort()|	对列表中的元素进行排序。
list.reverse()	|倒排列表中的元素。
list.copy()	|返回列表的浅复制，等于a[:]。


## 列表推导式
列表推导式提供了从序列创建列表的简单途径。通常应用程序将一些操作应用于某个序列的每个元素，用其获得的结果作为生成新列表的元素，或者根据确定的判定条件创建子序列。

每个列表推导式都在 for 之后跟一个表达式，然后有零到多个 for 或 if 子句。返回结果是一个根据表达从其后的 for 和 if 上下文环境中生成出来的列表。如果希望表达式推导出一个元组，就必须使用括号。
```python
>>> vec = [2, 4, 6]
>>> [3*x for x in vec]
# [6, 12, 18]
>>> [[x, x**2] for x in vec]
# [[2, 4], [4, 16], [6, 36]]

```

## 嵌套列表解析
Python的列表还可以嵌套。

以下实例展示了3X4的矩阵列表：

```python
>>> matrix = [
     [1, 2, 3, 4],
     [5, 6, 7, 8],
     [9, 10, 11, 12],]
```
以下实例将3X4的矩阵列表转换为4X3列表：
```python
>>> [[row[i] for row in matrix] for i in range(4)]
[[1, 5, 9], [2, 6, 10], [3, 7, 11], [4, 8, 12]]
```
以下实例也可以使用以下方法来实现：
```python
>>> transposed = []
>>> for i in range(4):
        transposed.append([row[i] for row in matrix])

>>> transposed
[[1, 5, 9], [2, 6, 10], [3, 7, 11], [4, 8, 12]]
```
另外一种实现方法：
```python
>>> transposed = []
>>> for i in range(4):
     # the following 3 lines implement the nested listcomp
     transposed_row = []
     for row in matrix:
         transposed_row.append(row[i])
     transposed.append(transposed_row)

>>> transposed
[[1, 5, 9], [2, 6, 10], [3, 7, 11], [4, 8, 12]]
```

## 文件I/O
### 读取键盘输入
Python提供了两个内置函数从标准输入读入一行文本，默认的标准输入是键盘。如下：

- raw_input
- input

#### 1. raw_input函数
raw_input([prompt]) 函数从标准输入读取一个行，并返回一个字符串（去掉结尾的换行符）：

```python
str = raw_input("请输入：")
print "你输入的内容是: ", str
# 这将提示你输入任意字符串，然后在屏幕上显示相同的字符串。当我输入"Hello Python！"，它的输出如下：
# 请输入：Hello Python！
# 你输入的内容是:  Hello Python！
```
#### 2. input函数
input([prompt]) 函数和 raw_input([prompt]) 函数基本类似，但是 input 可以接收一个Python表达式作为输入，并将运算结果返回。
```python
str = input("请输入：")
print "你输入的内容是: ", str
# 这会产生如下的对应着输入的结果：
# 请输入：[x*5 for x in range(2,10,2)]
# 你输入的内容是:  [10, 20, 30, 40]
```

## 打开和关闭文件
现在，您已经可以向标准输入和输出进行读写。现在，来看看怎么读写实际的数据文件。

Python 提供了必要的函数和方法进行默认情况下的文件基本操作。你可以用 file 对象做大部分的文件操作。

#### 1. open 函数
你必须先用Python内置的open()函数打开一个文件，创建一个file对象，相关的方法才可以调用它进行读写。

语法：
`file object = open(file_name [, access_mode][, buffering])`
各个参数的细节如下：

- file_name：file_name变量是一个包含了你要访问的文件名称的字符串值。
- access_mode：access_mode决定了打开文件的模式：只读，写入，追加等。所有可取值见如下的完全列表。这个参数是非强制的，默认文件访问模式为只读(r)。
- buffering:如果buffering的值被设为0，就不会有寄存。如果buffering的值取1，访问文件时会寄存行。如果将buffering的值设为大于1的整数，表明了这就是的寄存区的缓冲大小。如果取负值，寄存区的缓冲大小则为系统默认。

```python
fo = open("foo.txt", "w")
print "文件名: ", fo.name
print "是否已关闭 : ", fo.closed
print "访问模式 : ", fo.mode
print "末尾是否强制加空格 : ", fo.softspace
'''
文件名:  foo.txt
是否已关闭 :  False
访问模式 :  w
末尾是否强制加空格 :  0
'''
```

#### 2. close()方法
File 对象的 close（）方法刷新缓冲区里任何还没写入的信息，并关闭该文件，这之后便不能再进行写入。

当一个文件对象的引用被重新指定给另一个文件时，Python 会关闭之前的文件。用 close（）方法关闭文件是一个很好的习惯。

#### 3. write()方法
write()方法可将任何字符串写入一个打开的文件。需要重点注意的是，Python字符串可以是二进制数据，而不是仅仅是文字。

write()方法不会在字符串的结尾添加换行符('\n')：

语法：
`fileObject.write(string)`
在这里，被传递的参数是要写入到已打开文件的内容。

例子：
```python
# 打开一个文件
fo = open("foo.txt", "w")
fo.write( "www.runoob.com!\nVery good site!\n")
 
# 关闭打开的文件
fo.close()
```

#### 4. read()方法
read（）方法从一个打开的文件中读取一个字符串。需要重点注意的是，Python字符串可以是二进制数据，而不是仅仅是文字。

语法：
`fileObject.read([count])`
在这里，被传递的参数是要从已打开文件中读取的字节计数。该方法从文件的开头开始读入，如果没有传入count，它会尝试尽可能多地读取更多的内容，很可能是直到文件的末尾。

```python
# 打开一个文件
fo = open("foo.txt", "r+")
str = fo.read(10)
print "读取的字符串是 : ", str
# 关闭打开的文件
fo.close()
```
以上实例输出结果：
`读取的字符串是 :  www.runoob`


## 重命名和删除文件
Python的os模块提供了帮你执行文件处理操作的方法，比如重命名和删除文件。

要使用这个模块，你必须先导入它，然后才可以调用相关的各种功能。

#### 1. rename() 方法
rename() 方法需要两个参数，当前的文件名和新文件名。

语法：
`os.rename(current_file_name, new_file_name)`

#### 2. remove()方法
你可以用remove()方法删除文件，需要提供要删除的文件名作为参数。

语法：
`os.remove(file_name)`


## 目录

#### 1. mkdir()
可以使用os模块的mkdir()方法在当前目录下创建新的目录们。你需要提供一个包含了要创建的目录名称的参数。

语法：
`os.mkdir("newdir")`

#### 2. chdir()
可以用chdir()方法来改变当前的目录。chdir()方法需要的一个参数是你想设成当前目录的目录名称。

语法：
`os.chdir("newdir")`

#### 3. getcwd()方法：
getcwd()方法显示当前的工作目录。

语法：
`os.getcwd()`

#### 4. rmdir()方法
rmdir()方法删除目录，目录名称以参数传递。
在删除这个目录之前，它的所有内容应该先被清除。

语法：
`os.rmdir('dirname')`



















>>>>>>> 285b814601d5399a58128ce8798b99dda8ed6d59
