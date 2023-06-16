# 一、数据采集和处理

## 1，采集图片

## 2，制作标签

采集好数据后需要制作VOC格式的数据集。利用 `labelImg` 进行数据标注。

进入 `labelImg` 文件夹后，首先在 `data/predefined_classes.txt` 文件中编辑类别名称

运行 `labelImg.py` 程序， 点击`打开目录`选择图片目录，并点击`改变存放目录`选择标签存放目录， 并切换到  `VOC`  格式。  

注意标注数据尽量不要包括多余的区域，防止后续可能出现的干扰。快捷键 `W  `键创建区块， `A` 键向前翻， `D` 键向后翻，注意保存已标注的结果。每标注一张则会产生一个对应的 `xml` 文件。  

## 3，划分数据

首先创建如下目录， `annotations` 文件夹存放 `xml` 文件， `img_lab/images` 存放图片， `img_lab/classes.txt` 包含所有类别名， 与上文中 `predefined_classes.txt` 内容一致。运行 `split_train_val.py` 程序可划分训练集、 验证集、 测试集， 并在 `img_lab` 文件夹下生成 `txt` 文件， 内含加载路径。（代码内可更改训练集在总数据集中占比，默认为 `0.9`）    

# 二、模型训练