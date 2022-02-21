<<<<<<< HEAD
---
title: python学习-5
author: xmcchv
date: 2021-07-07 19:01:38
tags: python
---
## CGI编程
CGI(Common Gateway Interface)，通用网关接口，它是一段程序，运行在服务器上如：HTTP 服务器，提供同客户端 HTML 页面的接口。
![CGI架构图](/images/Cgi01.png "CGI架构图")

## 操作数据库

#### 1. pymysql

- 若出现问题`pymysql连接数据库报错TypeError: __init__() takes 1 positi`
原因：pymsql版本变化，参数名必须写完整
```python
conn=pymysql.connect(host="localhost",user="root",
    password="123456",database="onlinecourse",charset="utf8")
```
- 封装pymysql模块
```python
# databasehandler模块
import pymysql

class DataBaseHandler(object):
    def __init__(self,hostname,username,passwd,db,port,cset="utf8"):
        self.db=pymysql.connect(host=hostname,user=username,password=passwd,database=db,charset=cset,port=port)
        self.cursor = self.db.cursor()

    def __close__(self):
        self.db.close()

    # select语句
    def __sel_exec__(self,sql):
        # 获取游标
        self.cursor=self.db.cursor()
        # 开启事务
        try:
            self.cursor.execute(sql)
            data=self.cursor.fetchall()
            # 提交
            self.db.commit()
            return data
        except:
            # 回滚
            self.db.rollback()
        finally:
            # 关闭游标
            self.cursor.close()

    # insert update delete语句
    def __not_sel_exec__(self,sql):
        self.cursor=self.db.cursor()
        try:
            result=self.cursor.execute(sql)
            self.db.commit()
            return result
        except:
            self.db.rollback()
        finally:
            self.cursor.close()

# 测试文件
import DataBaseHandler
# 打开数据库连接
# conn=pymysql.connect(host="localhost",user="root",password="123456",database="onlinecourse",charset="utf8")
db=DataBaseHandler.DataBaseHandler("localhost","root","123456","onlinecourse",3306)
# print(cursor.execute("select * from edu_course_teacher",None))
rs=db.__sel_exec__("select * from edu_user")
print(rs)
rs=db.__sel_exec__("select * from edu_course_teacher")
print(rs)
db.__close__()
```


#### 2. 数据库查询操作
Python查询Mysql使用 fetchone() 方法获取单条数据, 使用fetchall() 方法获取多条数据。

- fetchone(): 该方法获取下一个查询结果集。结果集是一个对象
- fetchall():接收全部的返回结果行.
- rowcount: 这是一个只读属性，并返回执行execute()方法后影响的行数。


#### 3. 事务的ACID性质

事务机制可以确保数据一致性。
事务应该具有4个属性：原子性、一致性、隔离性、持久性。这四个属性通常称为ACID特性。
- 原子性（atomicity）。一个事务是一个不可分割的工作单位，事务中包括的诸操作要么都做，要么都不做。
- 一致性（consistency）。事务必须是使数据库从一个一致性状态变到另一个一致性状态。一致性与原子性是密切相关的。
- 隔离性（isolation）。一个事务的执行不能被其他事务干扰。即一个事务内部的操作及使用的数据对并发的其他事务是隔离的，并发执行的各个事务之间不能互相干扰。
- 持久性（durability）。持续性也称永久性（permanence），指一个事务一旦提交，它对数据库中数据的改变就应该是永久性的。接下来的其他操作或故障不应该对其有任何影响。
































































































=======
---
title: python学习-5
author: xmcchv
date: 2021-07-07 19:01:38
tags: python
---
## CGI编程
CGI(Common Gateway Interface)，通用网关接口，它是一段程序，运行在服务器上如：HTTP 服务器，提供同客户端 HTML 页面的接口。
![CGI架构图](/images/Cgi01.png "CGI架构图")

## 操作数据库

#### 1. pymysql

- 若出现问题`pymysql连接数据库报错TypeError: __init__() takes 1 positi`
原因：pymsql版本变化，参数名必须写完整
```python
conn=pymysql.connect(host="localhost",user="root",
    password="123456",database="onlinecourse",charset="utf8")
```
- 封装pymysql模块
```python
# databasehandler模块
import pymysql

class DataBaseHandler(object):
    def __init__(self,hostname,username,passwd,db,port,cset="utf8"):
        self.db=pymysql.connect(host=hostname,user=username,password=passwd,database=db,charset=cset,port=port)
        self.cursor = self.db.cursor()

    def __close__(self):
        self.db.close()

    # select语句
    def __sel_exec__(self,sql):
        # 获取游标
        self.cursor=self.db.cursor()
        # 开启事务
        try:
            self.cursor.execute(sql)
            data=self.cursor.fetchall()
            # 提交
            self.db.commit()
            return data
        except:
            # 回滚
            self.db.rollback()
        finally:
            # 关闭游标
            self.cursor.close()

    # insert update delete语句
    def __not_sel_exec__(self,sql):
        self.cursor=self.db.cursor()
        try:
            result=self.cursor.execute(sql)
            self.db.commit()
            return result
        except:
            self.db.rollback()
        finally:
            self.cursor.close()

# 测试文件
import DataBaseHandler
# 打开数据库连接
# conn=pymysql.connect(host="localhost",user="root",password="123456",database="onlinecourse",charset="utf8")
db=DataBaseHandler.DataBaseHandler("localhost","root","123456","onlinecourse",3306)
# print(cursor.execute("select * from edu_course_teacher",None))
rs=db.__sel_exec__("select * from edu_user")
print(rs)
rs=db.__sel_exec__("select * from edu_course_teacher")
print(rs)
db.__close__()
```


#### 2. 数据库查询操作
Python查询Mysql使用 fetchone() 方法获取单条数据, 使用fetchall() 方法获取多条数据。

- fetchone(): 该方法获取下一个查询结果集。结果集是一个对象
- fetchall():接收全部的返回结果行.
- rowcount: 这是一个只读属性，并返回执行execute()方法后影响的行数。


#### 3. 事务的ACID性质

事务机制可以确保数据一致性。
事务应该具有4个属性：原子性、一致性、隔离性、持久性。这四个属性通常称为ACID特性。
- 原子性（atomicity）。一个事务是一个不可分割的工作单位，事务中包括的诸操作要么都做，要么都不做。
- 一致性（consistency）。事务必须是使数据库从一个一致性状态变到另一个一致性状态。一致性与原子性是密切相关的。
- 隔离性（isolation）。一个事务的执行不能被其他事务干扰。即一个事务内部的操作及使用的数据对并发的其他事务是隔离的，并发执行的各个事务之间不能互相干扰。
- 持久性（durability）。持续性也称永久性（permanence），指一个事务一旦提交，它对数据库中数据的改变就应该是永久性的。接下来的其他操作或故障不应该对其有任何影响。
































































































>>>>>>> 285b814601d5399a58128ce8798b99dda8ed6d59
