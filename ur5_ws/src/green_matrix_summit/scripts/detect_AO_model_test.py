#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
import message_filters
# from cv_bridge import CvBridge
import numpy as np
import os

from detectron2.engine import default_argument_parser
from human_segmentation import Detectron2Tensormask

class DetectAOModelNode:
    def __init__(self, args):

        # Setup necessary flags

        # ROS node setup
        rospy.init_node('detect_ao_model_node')
        self.full_save_path_root = '/home/RVLuser/data/scene_20230517' # change this
        # self.full_save_path_root = rospy.get_param('~save_path')

        # Save paths
        self.rgb_save_path = os.path.join(self.full_save_path_root, 'rgb')
        self.rgb_seg_save_path = os.path.join(self.full_save_path_root, 'rgb_seg')
        self.depth_seg_save_path = os.path.join(self.full_save_path_root, 'depth_seg')
        if not os.path.exists(self.rgb_save_path):
            os.makedirs(self.rgb_save_path)
        if not os.path.exists(self.rgb_seg_save_path):
            os.makedirs(self.rgb_seg_save_path)
        if not os.path.exists(self.depth_seg_save_path):
            os.makedirs(self.depth_seg_save_path)

        # TensorMask segmentation object
        self.d2t = Detectron2Tensormask(cfg_args=args, save_path_root=self.full_save_path_root)
        rospy.sleep(5)
        print('Setup over! You can start the camera capture process.')

        # ROS subscribers/publishers init
        self.rgb_topic = '/camera/color/image_raw'
        self.depth_topic = '/camera/aligned_depth_to_color/image_raw'
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=500, slop=2)
        ts.registerCallback(self.callback)
        self.bridge = CvBridge()


    def callback(self, rgb_msg, depth_msg):
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'passthrough')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

        # Save RGB image
        cv2.imwrite(self.rgb_save_path, rgb_image)

        # segmented RGB and depth images are saved inside this function
        self.d2t.segment_rgb_and_depth_images(rgb_image_in=rgb_image, depth_image_in=depth_image, visualize_images=True, save_images=True)

if __name__ == '__main__':
    # Get passed arguments from terminal
    args = default_argument_parser().parse_args()

    detect_node = DetectAOModelNode(args)

    rospy.spin()
