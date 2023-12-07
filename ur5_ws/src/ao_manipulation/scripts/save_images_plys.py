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
        self.fps_scaler = self.camera_fps / self.wanted_fps
        self.fps_scaler_counter = 0

        # Convert ROS Image message to OpenCV image
        self.bridge = CvBridge()

        # Save paths
        self.rgb_save_path = rospy.get_param('/rgb_save_path')
        self.rgb_seg_save_path = rospy.get_param('/rgb_seg_save_path')
        self.depth_save_path = rospy.get_param('/depth_save_path')
        self.depth_seg_save_path = rospy.get_param('/depth_seg_save_path')
        self.ply_save_path = rospy.get_param('/ply_save_path')

        self.reset_dirs(self.rgb_save_path)
        self.reset_dirs(self.rgb_seg_save_path)
        self.reset_dirs(self.depth_save_path)
        self.reset_dirs(self.depth_seg_save_path)
        self.reset_dirs(self.ply_save_path)

        # Detectron2 + TensorMask
        self.d2t = Detectron2Tensormask(args, rgb_save_path=self.rgb_seg_save_path, depth_save_path=self.depth_seg_save_path)

        rgb_image_sub = Subscriber(self.rgb_image_topic, Image)
        depth_image_sub = Subscriber(self.depth_image_topic, Image)
        camera_info_sub = Subscriber(self.camera_info_topic, CameraInfo)

        self.build_model_pub = rospy.Publisher('/build_model', Bool, queue_size=1)
        
        synchronizer = ApproximateTimeSynchronizer([rgb_image_sub, depth_image_sub, camera_info_sub], queue_size = 100, slop=0.1)
        synchronizer.registerCallback(self.callback)

        self.counter_images = 0
        self.space_pressed = 0
        rospy.spin()

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
        
        cv2.imshow('RGB image', rgb_image)
        key = cv2.waitKey(1)
        if key == 32: # space entered
            self.space_pressed += 1
        
        if self.space_pressed == 1:   
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')

            if depth_image.dtype == 'float32':
                depth_image = (depth_image * 1000.0).astype(np.uint16)  
                depth_image = np.round(depth_image).astype(np.uint16)
                normalized_depth = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_image_8bit = np.uint8(normalized_depth)

            cv2.imshow('Depth image', depth_image_8bit)
            
            cv2.imwrite(os.path.join(self.rgb_save_path, '%04d.png' % self.counter_images), rgb_image)
            cv2.imwrite(os.path.join(self.depth_save_path, '%04d.png' % self.counter_images), depth_image)

            self.counter_images += 1
        
        if self.space_pressed == 2:
            
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

                # # Open3D ply create
                # intrinsics = o3d.camera.PinholeCameraIntrinsic(
                #     width=camera_info_msg.width,
                #     height=camera_info_msg.height,
                #     fx=K[0, 0], # fx
                #     fy=K[1, 1], # fy
                #     cx=K[0, 2], # cx
                #     cy=K[1, 2] # cy
                #     )
                # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                #     o3d.geometry.Image(color_reversed),
                #     o3d.geometry.Image(depth_seg_img),
                #     depth_scale=1000.0,  # depth values are in millimeters
                #     depth_trunc=5.0,    # truncate depth values beyond 3.0 meters
                #     convert_rgb_to_intensity=False
                # )
                # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                #     rgbd_image,
                #     intrinsics
                #     )
                
                # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                # # pcd_with_normals = o3d.geometry.estimate_normals(pcd, search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                # mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd)
                
                # # o3d.io.write_point_cloud(os.path.join(self.ply_save_path, '%04d.ply' % i), pcd)
                # o3d.io.write_triangle_mesh(os.path.join(self.ply_save_path, '%04d.ply' % i), mesh)


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