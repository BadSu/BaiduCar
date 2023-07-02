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

- 平台：`AI Studio`
- 检测模型：`yolov3_mobilenet_v1`

创建`Notebook`项目并配置环境如下：

- 项目框架 = `PaddlePaddle 2.2.2`
- 项目环境 = `python 3.7`

启动环境（）后，首先克隆`PaddleDetection`训练仓库，新建一个终端页面，并在终端键入以下命令：

```bash
git clone https://github.com/PaddlePaddle/PaddleDetection.git -b release/2.4
```

克隆完成后， 切换` PaddleDetection` 目录  

```bash
cd PaddleDetection
```

 并安装必要依赖：

```bash
pip install -r requirements.txt
```

执行编译：

```bash
python setup.py install
```

至此， `PaddleDetection` 训练环境搭建完毕.  

把之前制作好的数据集整个打包， 直接上传， 右键解压即可。 数据集解压至 `PaddleDetection/dataset` 路径下,再运行以下指令解压：  

```bash
unzip -oqd /home/aistudio/PaddleDetection/dataset/ /home/aistudio/data/data186919/baidu_car0311.zip
```

解压完成后进入目录， 运行` split_train_val `程序以划分数据集：  
