0. 原始数据为 .png 格式，大小为 1280 x 720
    需要vgg.npy文件

1. 标注数据 工具Labelme.exe
标注完成生成对应.json文件，与图片在同一目录
下载地址：https://github.com/wkentaro/labelme/releases
    1. Open Dir 打开包含图片的文件夹
    2. Edit --> Create lineStrip 
    3. 点击划线，标注为1、2
    4. 保存 ctrl + s 或者 Flie --> Save Automatic


2. 需要生成三个文件夹 gt_image_binary gt_image_instance image
    1. 前两个由 batch_label1.py 生成
    2. image 保存的是 原图

3. 划分训练集和验证集
    batch_label1.py脚本会生成一个 val.txt 文件，手动创建 train.txt，拷贝内容

4. 训练
    train_lanenet1.py 脚本 
    可以从头训练可以加载预训练模型