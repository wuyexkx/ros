#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import os
import sys
# import argparse
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import torch
from torchvision import transforms

cur_path = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(cur_path, '..'))

from model.lednet import LEDNet
import utils as ptutil
import numpy as np
import time
import cv2


if __name__ == '__main__':

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = LEDNet(19).to(device)
    model.load_state_dict(torch.load('./LEDNet_final.pth', 
            map_location=None if torch.cuda.is_available() else 'cpu'))

    model.eval()
    # Transform
    transform_fn = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

    cap = cv2.VideoCapture('./png/demo.mkv') 
    cv2.namedWindow('row image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('seg image', cv2.WINDOW_NORMAL)
    while(cap.isOpened()): 
        ret, img = cap.read() 
        cv2.imshow('row image', img) 
    # ----------------------------------------
        time_start=time.time() # =============time 
        # Load Images
        # img = Image.open(os.path.join(cur_path, 'png/demo.png'))
        # if img.mode == 'RGBA': img = img.convert('RGB')

        img = transform_fn(img).unsqueeze(0).to(device) # 添加一个维度
        with torch.no_grad():
            output = model(img)

        predict = torch.argmax(output, 1).squeeze(0).cpu().numpy()
        mask = ptutil.get_color_pallete(predict, 'citys') # P mode image

        print('totally cost: {:.3f}ms'.format((time.time()-time_start) * 1000)) # ==============time

        # k = cv2.waitKey(1) # 等待1ms然后继续往下执行
        #q键退出
        # if (k & 0xff == ord('q')): 
        #     break

        mask= mask.convert("RGB")
        cv2.imshow('seg image', np.array(mask))
    # ----------------------------------------

    cap.release() 
    cv2.destroyAllWindows()
