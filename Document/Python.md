# 一、输出和运算符

## 1.1 程序和输出

由于在程序运行的过程中我们不知道干到了那一步，正在进入哪一段逻辑，为了查看程序内部运行情况，需要程序向外部告知，告知的方式其实就是输出`日志`。

### 1.1.1 打印输出

程序可以通过`print`输出函数，将程序执行的数据输出到控制台

```python
print('hello world')
```

## 1.2 注释

1.2.1 单行注释

```python
# 这是一个注释，不参与运行
```
1.2.2 多行注释

```python
'''
这是一个多行注释，不参与运行
'''
```

## 1.3 基本数学运算

### 1.3.1 加法

```python
1 + 1
```

### 1.3.2 减法

```python
1 - 1
```

### 1.3.3 乘法

```python
1 * 1
```
### 1.3.4 除法

```python
1 / 1
```

### 1.3.5 取余数

```python
1 % 1
```
### 1.3.6 取幂

```python
3 ** 2
```

### 1.3.7 整数除法（省略小数点）

```python
3 // 2
```

# 二、变量和数据类型

## 2.1 变量

**变量**是用来描述计算机中的**数据存储空间**的。我们可以通过变量来保存定义的数据。

### 2.1.1 变量的定义

规则： `变量名`=`存储的值`

例如，我定义了一个变量age，用来存储一个数字：

```python
age = 18
```

### 2.1.2 变量的命名规则

- 变量名只能由数字，字母下划线组成
- 不能用数字开头
- 不能是关键字
- 区分大小写

### 2.1.3 变量命名规范

我们在编写python代码时，通常采用下划线命名法（推荐使用）

## 2.2 数据类型 

- 数字
  - 整数
  - 小数
- 布尔 True：False
- 字符串
  
### 2.2.1 多个变量赋值

```python
name, age, gender = 'Alex',20,True
```

#### 数字的运算

```python
age = 5
age = age + 5
age += 5
age -= 5
age *= 5
age /= 5
```

#### type函数

可以得到变量的数据类型

```python
x = 4
print(type(x))
```

输出结果如下

```python
<class 'int'>
```

## 三、列表

列表是一个序列（sequence），我们可以理解为一个装数据的容器。其中的每一个数据我们称为**元素**

列表的类型为`List`，用一对`[]`表示

```python
names = ['Alex','Eric','Dave']
```

### 3.1 列表操作

打印列表长度

```python
print(len(names))
```

### 3.2 访问元素

通过下标进行访问，**列表下标从0开始**

可以用访问负数（当前下标减去长度）

### 3.3 添加元素 insert

```python
names = ['Alex','Eric','Dave']
names.insert(1,'test')
print(names)
```

得到输出如下

```python
['Alex','Test','Eric','Dave']
```

### 3.3 删除元素 根据内容进行删除

```python
names = ['Alex','Eric','Dave']
names.remove('Dave')
print(names)
```

得到输出如下

```python
['Alex','Eric']
```


### 3.4 删除元素 根据索引进行删除

```python
names = ['Alex','Eric','Dave']
del names(2)
print(names)
```

得到输出如下

```python
['Alex','Eric']
```

### 3.5 修改

```python
names = ['Alex','Eric','Dave']
names[1] = 'test'
print(names)
```

得到输出如下

```python
['Alex','Test','Eric']
```

### 3.6 索引查询

```python
names = ['Alex','Eric','Dave']
index = names.index('Alex')
print(index)
```

得到输出如下

```python
1
```

### 3.7 排序 升

```python
nums = ['10','8','5','90','32']
names.sort ()
print(nums)
```

得到输出如下

```python
['5','8','10','32','90']
```

### 3.7 排序 降


```python
nums = ['10','8','5','90','32']
names.sort (reverse=true)
print(nums)
```

得到输出如下

```python
['90','32','10','8','5']
```

### 3.8 反转

```python
names = ['Alex','Eric','Dave']
names.reverse()
print(names)
```

得到输出如下

```python

['Dave','Eric','Alex']

```


