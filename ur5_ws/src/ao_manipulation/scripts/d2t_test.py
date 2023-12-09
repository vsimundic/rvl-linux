#!/usr/bin/env python

from human_segmentation import Detectron2Tensormask
from detectron2.engine import default_argument_parser
import open3d as o3d
import os
import cv2

if __name__ == '__main__':
    args = default_argument_parser().parse_args()
    
    rgb_load_path = '/home/RVLuser/data/summit_test/rgb'
    depth_load_path = '/home/RVLuser/data/summit_test/depth'
    
    num_images = 52
    
    rgb_seg_save_path = '/home/RVLuser/data/summit_test/rgb_seg'
    depth_seg_save_path = '/home/RVLuser/data/summit_test/depth_seg'

    d2t = Detectron2Tensormask(args, rgb_save_path=rgb_seg_save_path, depth_save_path=depth_seg_save_path)
    
    for i in range(num_images):
        print(i)
        rgb_image = cv2.imread(os.path.join(rgb_load_path, '%04d.png' % i), cv2.IMREAD_COLOR)
        depth_image = cv2.imread(os.path.join(depth_load_path, '%04d.png' % i), cv2.IMREAD_UNCHANGED)
        
        color_reversed = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        _, _ = d2t.segment_rgb_and_depth_images(color_reversed, depth_image, i, False, True)
