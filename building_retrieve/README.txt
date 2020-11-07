# building_retrieve
保存数据库（utils/save_database.cpp） 和 检索图片（utils/img_retrieve.cpp）


1. save_database 相关参数设置及位置
    都在save_database.cpp里
        图片切分 和 resize 参数 在第51行
        配置聚类树的分支树(k)，以及深度(l)在 第100行

2. img_retrieve
    图片切分 和 resize 参数(在my_function.h的第34行)


3. 重命名后用于产生数据库的图片放在images下
4. 中间产生的三个.gz/.txt/.gz都在database文件夹下
5. 在检索中只用到mould_db.yml.gz和mould_names.txt
