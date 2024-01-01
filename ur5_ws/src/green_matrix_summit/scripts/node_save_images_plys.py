#!/usr/bin/env python

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
import os
import numpy as np
import RVLPYRGBD2PLY
import rosnode
from human_segmentation import Detectron2Tensormask
from detectron2.engine import default_argument_parser
import open3d as o3d
import rosbag

class ImageSaver():
    def __init__(self, args):
        self.node_name = 'save_images_node'

        # Initialize ROS node
        rospy.init_node(self.node_name)

        # Subscriber topics
        self.rgb_image_topic = rospy.get_param('/rgb_image_topic')
        self.depth_image_topic = rospy.get_param('/depth_image_topic')
        self.camera_info_topic = rospy.get_param('/camera_info_topic')
        self.camera_fps = rospy.get_param('/camera_fps')
        self.wanted_fps = rospy.get_param('/wanted_fps')
        self.bag_used = rospy.get_param('~bag_used')
        self.fps_scaler = self.camera_fps / self.wanted_fps
        self.fps_scaler_counter = 0

        self.build_model_pub = rospy.Publisher('/build_model', Bool, queue_size=1)
        self.start_saving_sub = rospy.Subscriber('/save_images_start', Bool, self.start_callback)

        self.is_skip = rospy.get_param('~skip')

        # Convert ROS Image message to OpenCV image
        self.bridge = CvBridge()

        # Save paths
        self.rgb_save_path = rospy.get_param('/rgb_save_path')
        self.rgb_seg_save_path = rospy.get_param('/rgb_seg_save_path')
        self.depth_save_path = rospy.get_param('/depth_save_path')
        self.depth_seg_save_path = rospy.get_param('/depth_seg_save_path')
        self.ply_save_path = rospy.get_param('/ply_save_path')

        # Detectron2 + TensorMask
        self.d2t = Detectron2Tensormask(args, rgb_save_path=self.rgb_seg_save_path, depth_save_path=self.depth_seg_save_path)

        rospy.spin()


    def start_callback(self, msg):
        if not self.is_skip:
            if msg:
                if not self.bag_used:
                    self.reset_dirs(self.rgb_save_path)
                    self.reset_dirs(self.rgb_seg_save_path)
                    self.reset_dirs(self.depth_save_path)
                    self.reset_dirs(self.depth_seg_save_path)
                    self.reset_dirs(self.ply_save_path)

                    rgb_image_sub = Subscriber(self.rgb_image_topic, Image)
                    depth_image_sub = Subscriber(self.depth_image_topic, Image)
                    camera_info_sub = Subscriber(self.camera_info_topic, CameraInfo)
                    self.subscribers = [rgb_image_sub, depth_image_sub, camera_info_sub]
                    self.synchronizer = ApproximateTimeSynchronizer(self.subscribers, queue_size = 100, slop=0.1)
                    self.synchronizer.registerCallback(self.callback)

                    self.counter_images = 0
                    self.space_pressed = 0
                else:
                    self.bag_path = rospy.get_param('/bag_path')
                    self.process_bag(self.bag_path)
        else:
            rospy.sleep(2)
            # Send signal to another node
            b_build_model = Bool()
            b_build_model.data = True
            self.build_model_pub.publish(b_build_model)

            # rosnode.kill_nodes([self.node_name])
            rospy.signal_shutdown('Saving PLYs node done. Shutting down.')


    def process_bag(self, bag_path):
        self.reset_dirs(self.rgb_save_path)
        self.reset_dirs(self.rgb_seg_save_path)
        self.reset_dirs(self.depth_save_path)
        self.reset_dirs(self.depth_seg_save_path)
        self.reset_dirs(self.ply_save_path)

        bag = rosbag.Bag(bag_path)
        rgb_counter = 0
        depth_counter = 0
        save_rgb_counter = 0
        save_depth_counter = 0
        fps_scaler = round(self.camera_fps / self.wanted_fps)
        b_check_K = False
        K = ''

        for topic, msg, _ in bag.read_messages(topics=[self.rgb_image_topic, self.depth_image_topic, self.camera_info_topic]):
            if topic == self.rgb_image_topic:
                rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                # cv2.imshow('RGB image', rgb_image)
                # cv2.waitKey(1)
                rgb_counter +=1
                if rgb_counter % fps_scaler == 0:
                    cv2.imwrite(os.path.join(self.rgb_save_path, '%04d.png' % save_rgb_counter), rgb_image)
                    save_rgb_counter += 1
            elif topic == self.depth_image_topic:
                depth_counter += 1
                if depth_counter % fps_scaler == 0:
                    depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

                    if depth_image.dtype == 'float32':
                        depth_image = (depth_image * 1000.0).astype(np.uint16)  
                        depth_image = np.round(depth_image).astype(np.uint16)
                        normalized_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                        depth_image_8bit = np.uint8(normalized_depth)
                    
                    cv2.imwrite(os.path.join(self.depth_save_path, '%04d.png' % save_depth_counter), depth_image)
                    save_depth_counter += 1
            elif topic == self.camera_info_topic and not b_check_K:
                b_check_K = True
                K = np.array(msg.K).reshape([3, 3])
        
        cv2.destroyAllWindows()
        rgbd2ply = RVLPYRGBD2PLY.RGBD2PLY()

        rospy.loginfo('Saving %d PLY files...' % save_rgb_counter)
        
        for i in range(save_rgb_counter):

            rospy.loginfo('Saving ply file %d...' % i)
            color_img = cv2.imread(os.path.join(self.rgb_save_path, '%04d.png' % i), cv2.IMREAD_COLOR)
            depth_img = cv2.imread(os.path.join(self.depth_save_path, '%04d.png' % i), cv2.IMREAD_UNCHANGED)
            
            color_reversed = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            _, depth_seg_img = self.d2t.segment_rgb_and_depth_images(color_reversed, depth_img, i, False, True)

            rgbd2ply.pixel_array_to_ply(color_img, 
                                        depth_seg_img, 
                                        K[0, 0], # fx
                                        K[1, 1], # fy
                                        K[0, 2], # cx
                                        K[1, 2], # cy
                                        color_img.shape[1], # width
                                        color_img.shape[0], # height
                                        0.050,
                                        os.path.join(self.ply_save_path, '%04d.ply' % i))
        
        rospy.loginfo('PLY files saved.')
        
        # Send signal to another node
        b_build_model = Bool()
        b_build_model.data = True
        self.build_model_pub.publish(b_build_model)

        # rosnode.kill_nodes([self.node_name])
        rospy.signal_shutdown('Saving PLYs node done. Shutting down.')


    def reset_dirs(self, dirpath):  
        if not os.path.exists(dirpath):
            os.makedirs(dirpath)
            return

        for filename in os.listdir(dirpath):
            if os.path.isfile(os.path.join(dirpath, filename)):
                os.remove(os.path.join(dirpath, filename))

    def callback(self, rgb_msg, depth_msg, camera_info_msg):

        self.fps_scaler_counter += 1
        if not self.fps_scaler_counter % self.fps_scaler == 0:
            return

        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_image.dtype == 'float32':
            depth_image = (depth_image * 1000.0).astype(np.uint16)  
            depth_image = np.round(depth_image).astype(np.uint16)
            normalized_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_8bit = np.uint8(normalized_depth)
        
        cv2.imshow('RGB image', rgb_image)
        cv2.imshow('Depth image', depth_image_8bit)
        key = cv2.waitKey(1)
        if key == 32: # space entered
            self.space_pressed += 1
        
        if self.space_pressed == 1:   

            
            cv2.imwrite(os.path.join(self.rgb_save_path, '%04d.png' % self.counter_images), rgb_image)
            cv2.imwrite(os.path.join(self.depth_save_path, '%04d.png' % self.counter_images), depth_image)

            self.counter_images += 1
        
        if self.space_pressed == 2:
            
            [sub.sub.unregister() for sub in self.subscribers]

            rospy.loginfo('Saving PLY files...')

            K = np.array(camera_info_msg.K).reshape([3, 3])

            rgbd2ply = RVLPYRGBD2PLY.RGBD2PLY()
            
            # for i in range(self.counter_images):
            #     color_img = cv2.imread(os.path.join(self.rgb_save_path, '%04d.png' % i), cv2.IMREAD_COLOR)
            #     depth_img = cv2.imread(os.path.join(self.depth_save_path, '%04d.png' % i), cv2.IMREAD_ANYDEPTH)


            rospy.loginfo('Number of images saved: %d' % self.counter_images)
            for i in range(self.counter_images):
                rospy.loginfo('Saving ply file %d...' % i)
                color_img = cv2.imread(os.path.join(self.rgb_save_path, '%04d.png' % i), cv2.IMREAD_COLOR)
                depth_img = cv2.imread(os.path.join(self.depth_save_path, '%04d.png' % i), cv2.IMREAD_UNCHANGED)
                
                color_reversed = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
                _, depth_seg_img = self.d2t.segment_rgb_and_depth_images(color_reversed, depth_img, i, False, True)

                rgbd2ply.pixel_array_to_ply(color_img, 
                                            depth_seg_img, 
                                            K[0, 0], # fx
                                            K[1, 1], # fy
                                            K[0, 2], # cx
                                            K[1, 2], # cy
                                            camera_info_msg.width, 
                                            camera_info_msg.height, 
                                            0.050,
                                            os.path.join(self.ply_save_path, '%04d.ply' % i))
            
            rospy.loginfo('PLY files saved.')
            
            # Send signal to another node
            b_build_model = Bool()
            b_build_model.data = True
            self.build_model_pub.publish(b_build_model)

            # rosnode.kill_nodes([self.node_name])
            rospy.signal_shutdown('Saving PLYs node done. Shutting down.')

if __name__ == '__main__':
    args = default_argument_parser().parse_args()
    ImageSaver(args)