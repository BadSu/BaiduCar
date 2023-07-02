## 使用说明
### 解压模型包
unzip wanteng.zip  
bottle pcb pcba scratch为四个不同模型的目录，每个模型有对应的测试图片xxx_images  
修改各个模型子目录里的config.json中的模型路径
### C++接口测试
在build中执行：cmake .. && make  
./ppnc_smart_detection ../pcb/config.json ../pcb_images/1.jpg
### Python接口测试
python3 infer_demo_batch.py ./pcb/config.json ./pcb_images/
