#!/usr/bin/env python3.8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import time
import numpy as np
import RVLPYRGBD2PLY as rvl
import os.path
from rosnode import kill_nodes
from PIL import Image as PilImage 
import RVLPYDDDetector 
import detectron2

numberOfImages = 0
keyCounter = 0

def stop_node(node_name):
    kill_nodes([node_name])

def image_callback(rgb,depth):
    global numberOfImages, keyCounter

    # Convert ROS Image message to OpenCV image
    bridge = CvBridge()
    cv_image_rgb = bridge.imgmsg_to_cv2(rgb, "bgr8")
    cv_image_depth = bridge.imgmsg_to_cv2(depth, "passthrough")
    

    # Display the image
    cv2.imshow("RGB",cv_image_rgb)
    #cv2.imshow("Depth",cv_image_depth)
   
    
    key = cv2.waitKey(1)
    if key == 32:
        keyCounter +=1
        
    if keyCounter == 1:
        cv2.imshow("RGB",cv_image_rgb)
        #cv2.imshow("Depth",cv_image_depth)
        fileNameRGB = str(numberOfImages).zfill(4) + ".png"
        fileNameDEPTH = str(numberOfImages).zfill(4)  + ".png"
        cv2.imwrite("/home/RVLuser/catkin_ws/src/Diplomski_V1/RGB_images/" + fileNameRGB, cv_image_rgb)
        cv2.imwrite("/home/RVLuser/catkin_ws/src/Diplomski_V1/DEPTH_images/" + fileNameDEPTH, cv_image_depth)
        numberOfImages +=1
    if keyCounter >= 2:
        cv2.destroyAllWindows()
        createPlyImages(numberOfImages)


def createPlyImages(counter):
    print('starting PLY...')

    rgbd2ply = rvl.RGBD2PLY()
    image_directory_name = '/home/RVLuser/catkin_ws/src/Diplomski_V1/'
    rgb_directory_name = 'RGB_images'
    depth_directory_name = 'DEPTH_images'
    ply_directory_name = 'PLY_seg'

    for i in range(0,counter):
        image_name = str(i).zfill(4)
        rgb_png_file_name = os.path.join(image_directory_name, rgb_directory_name, image_name + '.png')
        depth_png_file_name = os.path.join(image_directory_name, depth_directory_name, image_name + '.png')
        ply_file_name = os.path.join(image_directory_name, ply_directory_name, image_name + '.ply')

        # Create PLY from numpy arrays.

        rgb_img = PilImage.open(rgb_png_file_name)
        rgb_array = np.array(rgb_img.getdata()).astype(np.byte).copy()
        bgr_array = np.stack((rgb_array[:,2], rgb_array[:,1], rgb_array[:,0]), axis=1)
        depth_img = PilImage.open(depth_png_file_name)
        depth_array = np.array(depth_img.getdata()).astype(np.short).copy()
        rgbd2ply.pixel_array_to_ply(bgr_array, depth_array, 
            597.9033203125,
            598.47998046875,
            323.8436584472656,
            236.32774353027344,
            640,
            480,
            0.050,
        ply_file_name)
    print('completed PLY.')  
    stop_node("rgbAndDepth_image_subscriber")     


def main():
    rospy.init_node('rgbAndDepth_image_subscriber')
    rgb_image_sub = Subscriber('/camera/color/image_raw', Image)
    depth_image_sub = Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    
    synchronizer = ApproximateTimeSynchronizer([rgb_image_sub, depth_image_sub], queue_size = 5, slop=0.1)
    synchronizer.registerCallback(image_callback)

    rospy.spin()

    buildModel()

def buildModel():
    print('Starting...')

    detector = RVLPYDDDetector.PYDDDetector()
    detector.create('/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg')
    scene_sequence_dir_name = '/home/RVLuser/catkin_ws/src/Diplomski_V1/'
    ply_dir_name = 'PLY_seg'
    print('Loading mesh sequence %s' % scene_sequence_dir_name)
    first_mesh_id = 30
    last_mesh_id = 45#numberOfImages -1 

    for mesh_id in range(first_mesh_id, last_mesh_id):
        mesh_file_name = f'{mesh_id:04d}' + '.ply'
        print(mesh_file_name)
        mesh_path = os.path.join(scene_sequence_dir_name, ply_dir_name, mesh_file_name)
        print(mesh_path)
        detector.add_mesh(mesh_path)

    num_meshes = last_mesh_id - first_mesh_id + 1

    rgb_dir_name = 'RGB_images'
    print('Loading rgb sequence')
    for mesh_id in range(first_mesh_id, last_mesh_id + 1):
        rgb_file_name = f'{mesh_id:04d}' + '.png'
        print(rgb_file_name)
        rgb_path = mesh_path = os.path.join(scene_sequence_dir_name, rgb_dir_name, mesh_file_name)
        detector.add_rgb(rgb_path)

    # hyp_file_name = os.path.join(scene_sequence_dir_name, 'hyps.txt')
    # detector.set_hyp_file_name(hyp_file_name)
    ao = detector.detect()
    print(ao)

    print('completed build.')

if __name__ == '__main__':
    main()

