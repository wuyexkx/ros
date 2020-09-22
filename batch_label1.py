import json
import cv2
import numpy as np 
import os
import os.path as ops
def label(json_path):
    for raw_name in os.listdir(json_path):
        if not raw_name.endswith('.json'):
            continue  # 只读取json文件
        json_name = ops.join(json_path, raw_name) # 某个json的绝对路径
        raw_name = '{:s}.png'.format(raw_name.split('.')[0]) # .json --> .png
        
        bgi_path = ops.join(r'/home/dl4/tuzengbing/yanxiao/data/gt_image_binary',raw_name)
        igi_path = ops.join(r'/home/dl4/tuzengbing/yanxiao/data/gt_image_instance',raw_name) 
        f = open(json_name,encoding='utf-8')
        data = json.load(f)
        index = 0
        binary_img = np.zeros([720, 1280], np.uint8)
        instance_img = np.zeros([720, 1280], np.uint8)
        for line in  data['shapes']:
            points = line['points']
            points = np.array(points,np.int64)
            #print(points.shape)
            cv2.polylines(binary_img,[points],isClosed=False,
                                    color=255, thickness=5)
            cv2.polylines(instance_img,[points],isClosed=False,
                                    color= index*50+20, thickness=5)
            index+=1
        cv2.imwrite(bgi_path,binary_img)
        cv2.imwrite(igi_path,instance_img)


def gen_train_sample(src_dir, b_gt_image_dir, i_gt_image_dir, image_dir):
    """
    生成图像训练列表
    :param src_dir:
    :param b_gt_image_dir: 二值基准图
    :param i_gt_image_dir: 实例分割基准图
    :param image_dir: 原始图像
    :return:
    """

    with open('{:s}/training/train.txt'.format(src_dir), 'w') as file:

        for image_name in os.listdir(b_gt_image_dir):
            if not image_name.endswith('.png'):
                continue

            binary_gt_image_path = ops.join(b_gt_image_dir, image_name)
            instance_gt_image_path = ops.join(i_gt_image_dir, image_name)
            image_path = ops.join(image_dir, image_name)

            assert ops.exists(image_path), '{:s} not exist'.format(image_path)
            assert ops.exists(instance_gt_image_path), '{:s} not exist'.format(instance_gt_image_path)

            b_gt_image = cv2.imread(binary_gt_image_path, cv2.IMREAD_COLOR)
            i_gt_image = cv2.imread(instance_gt_image_path, cv2.IMREAD_COLOR)
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)

            if b_gt_image is None or image is None or i_gt_image is None:
                print('图像对: {:s}损坏'.format(image_name))
                continue
            else:
                info = '{:s} {:s} {:s}'.format(image_path, binary_gt_image_path, instance_gt_image_path)
                file.write(info + '\n')
    return

def sample():
    with open(r'/home/dl4/tuzengbing/yanxiao/data/val.txt','w') as file:
        for image_name in os.listdir(r'/home/dl4/tuzengbing/yanxiao/data/image'):
            if not image_name.endswith('.png'):
                continue
            bgi_path = ops.join(r'/home/dl4/tuzengbing/yanxiao/data/gt_image_binary',image_name)
            igi_path = ops.join(r'/home/dl4/tuzengbing/yanxiao/data/gt_image_instance',image_name)
            image_path = ops.join(r'/home/dl4/tuzengbing/yanxiao/data/image',image_name)
            info = '{:s} {:s} {:s}'.format(image_path, bgi_path, igi_path)
            file.write(info + '\n')
    return


# 第一步
json_path = r'/home/dl4/tuzengbing/yanxiao/label_img'
label(json_path) 

# 第二步
sample()
