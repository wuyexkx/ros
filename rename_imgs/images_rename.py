# coding=utf-8
import numpy as np
import shutil, os
from timeit import default_timer as timer



if __name__ == '__main__':
    # 模板路径
    mould_path = './moulds'
    # 特征保存路径
    save_path = './images'


# 重命名模板图片
    start = timer()
    bd_name_temp = "AAA"
    dict_temp = {}
    index = 0
    print("-----\nrename all images!-----\n")
    for parent, dirnames, names_list in os.walk(mould_path):
        if (names_list and names_list[0][-4:] == ".jpg"):  # names_list and
            for name in names_list:
                # 获取建筑名称
                bd_name = parent.split(os.path.sep)[-1]
                # 建筑序号重置条件
                if bd_name != bd_name_temp:
                    bd_name_temp = bd_name
                    dict_temp[bd_name] = 0
                    index = 0

                img_name = bd_name + "-" + str(index).zfill(5) + ".jpg"
                save_img = os.path.join(save_path, img_name)

                read_name = os.path.join(parent, name)
                shutil.copy(read_name, save_img)

                # print("index: " + str(index))
                print("from: " + read_name + " - to: " + save_img)

                index = index + 1
                dict_temp[bd_name] = index


    end = timer()
    num = 0
    print("\nbuilding name: num")
    for key in dict_temp:
        num = num + dict_temp[key]
        print(key +": " + str(dict_temp[key]))

    print("====Total num: " + str(num))
    print('\ntime: '+str(end-start)[:8]+' s')
