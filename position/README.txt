# master
图像匹配ros节点， 分支包括 1.对应原图重命名的py 2.数据库建立 和 图像检索cpp 

需要分支建立的两个文件：
  1.mould_names.txt
  2.mould_db.yml.gz
  
订阅消息"/new_camera/left_image" 摄像头图像
节点包编译： catkin_make -DCATKIN_WHITELIST_PACKAGES="position"
节点运行： rosrun position position 

修改参数在position的57行(Score最佳结果满足的最低score, cnt为检索7个结果中要>=4个)
if(ret[0].Score > 0.64 && cnt > 3) 

