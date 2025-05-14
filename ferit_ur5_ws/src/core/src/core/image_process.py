import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from core.ur5_commander import UR5Commander
import numpy as np
import cv2
import cv_bridge
import os
import RVLPYRGBD2PLY

class ImageProcess:
    def __init__(self, save_dir, rgb_topic, depth_topic, camera_info_topic, camera_fps, depth_encoding, robot_commander:UR5Commander=None):

        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic 
        self.camera_info_topic = camera_info_topic
        self.camera_fps = camera_fps
        self.depth_encoding = depth_encoding
        self.save_dir = save_dir


        self.bridge = cv_bridge.CvBridge()
        self.rgb_subscriber = None
        self.depth_subscriber = None
        self.ts = None
        self.camera_info_subscriber = None

        self.wanted_fps = 0
        self.__fps_scaler = 0
        self.__scaler_counter = 0
        self.image_counter = 0
        self.image_counter_one = 0

        self.__flag_saving = False

        self.robot = robot_commander
        self.__make_dirs()


    def __make_dirs(self):
        self.rgb_save_dir = os.path.join(self.save_dir, 'rgb')
        self.depth_save_dir = os.path.join(self.save_dir, 'depth')
        self.tool_poses_dir = os.path.join(self.save_dir, 'tool_poses')
        if not os.path.exists(self.rgb_save_dir):
            os.makedirs(self.rgb_save_dir)
        if not os.path.exists(self.depth_save_dir):
            os.makedirs(self.depth_save_dir)
        if self.robot is not None:
            if not os.path.exists(self.tool_poses_dir):
                os.makedirs(self.tool_poses_dir)
    

    def save_rgb_depth(self, wanted_fps):
        def callback(rgb_msg, depth_msg):
            self.__scaler_counter += 1
            if self.__scaler_counter % self.__fps_scaler == 0:
                self.__scaler_counter = 0
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=self.depth_encoding)

                if self.depth_encoding == '32FC1':
                    depth_image = np.nan_to_num(x=depth_image, nan=0.0)
                    depth_image = (depth_image * 1000).astype(np.uint16)
                ratio = np.amax(depth_image) / 256.
                depth_image_8bit = (depth_image/ratio).astype('uint8')
                depth_image_8bit = cv2.cvtColor(depth_image_8bit, cv2.COLOR_GRAY2BGR)
                both_images = np.concatenate([rgb_image, depth_image_8bit], axis=1)

                cv2.imshow('Images', both_images)
                key = cv2.waitKey(3)

                if key == ord('q'):
                    self.rgb_subscriber.sub.unregister()
                    self.depth_subscriber.sub.unregister()
                    rospy.loginfo('Done saving.')
                    cv2.destroyAllWindows()
                    rospy.signal_shutdown('done')

                elif key == ord('c'):
                    self.__flag_saving = not self.__flag_saving
                

                elif key == ord('p'):
                    self.__flag_saving = False

                    cv2.imwrite(os.path.join(self.rgb_save_dir, '%04d.png' % self.image_counter_one), rgb_image)
                    cv2.imwrite(os.path.join(self.depth_save_dir, '%04d.png' % self.image_counter_one), depth_image)
                    print('Image %04d saved.' %self.image_counter_one)
                    if self.robot is not None:
                        T_T_0_ = self.robot.get_current_tool_pose()
                        np.save(os.path.join(self.tool_poses_dir, 'T_T_0_%04d.npy' % self.image_counter_one), T_T_0_)
                    
                    self.image_counter_one += 1

                if self.__flag_saving:
                    cv2.imwrite(os.path.join(self.rgb_save_dir, '%04d.png' % self.image_counter), rgb_image)
                    cv2.imwrite(os.path.join(self.depth_save_dir, '%04d.png' % self.image_counter), depth_image)
                    
                    self.image_counter += 1
        

        self.image_counter = 0
        self.image_counter_one = 0

        self.__fps_scaler = int(self.camera_fps / wanted_fps)
        if self.rgb_topic is None and self.depth_topic is None:
            raise Exception('RGB and depth topics cannot be None.')
        
        self.rgb_subscriber = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_subscriber = message_filters.Subscriber(self.depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_subscriber, self.depth_subscriber], queue_size=500, slop=2)

        rospy.loginfo('Press q to exit.')
        rospy.loginfo('Press c to start/stop saving images.')
        rospy.loginfo('Press p to save one image.')
        self.ts.registerCallback(callback)
    
    # TODO: prepraviti
    def save_rgb(self, save_rgb_dir, wanted_fps):

        self.image_counter = 0
        self.__fps_scaler = int(self.camera_fps / wanted_fps)
        
        def callback(rgb_msg):

            self.__scaler_counter += 1
            if self.__scaler_counter % self.__fps_scaler == 0:
                self.__scaler_counter = 0
                rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')


                cv2.imshow('RGB image', rgb_image)
                key = cv2.waitKey(3)

                cv2.imwrite(os.path.join(save_rgb_dir, '%d.png' % self.image_counter), rgb_image)
                
                self.image_counter += 1

                if key == ord('q'):
                    rgb_subscriber.sub.unregister()
                    rospy.loginfo('Done saving.')


            rgb_subscriber = rospy.Subscriber(self.rgb_topic, Image, callback)

    # TODO: prepraviti
    def save_depth(self, save_depth, wanted_fps):

        self.image_counter = 0
        self.__fps_scaler = int(self.camera_fps / wanted_fps)
        
        def callback(depth_msg):

            self.__scaler_counter += 1
            if self.__scaler_counter % self.__fps_scaler == 0:
                self.__scaler_counter = 0
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=self.depth_encoding)

                ratio = np.amax(depth_image) / 256.
                depth_image_8bit = (depth_image/ratio).astype('uint8')
                depth_image_8bit = cv2.cvtColor(depth_image_8bit, cv2.COLOR_GRAY2BGR)
                cv2.imshow('RGB image', depth_image_8bit)
                key = cv2.waitKey(3)

                cv2.imwrite(os.path.join(save_depth, '%d.png' % self.image_counter), depth_image)
                
                self.image_counter += 1

                if key == ord('q'):
                    depth_subscriber.sub.unregister()
                    rospy.loginfo('Done saving.')


            depth_subscriber = rospy.Subscriber(self.rgb_topic, Image, callback)


    def save_camera_info(self):
        def callback(msg):
            with open(os.path.join(self.save_dir, 'camera_info.yaml'), 'w') as f:
                f.write(msg.__str__())
                rospy.loginfo('Saved camera information to file.')
                rospy.loginfo(msg.K)
            subscriber.unregister()

        subscriber = rospy.Subscriber(self.camera_info_topic, CameraInfo, callback)


class OneShotImageCapture:
    def __init__(self, save_dir, rgb_topic, depth_topic, camera_info_topic, depth_encoding):
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.depth_encoding = depth_encoding
        self.save_dir = save_dir

        self.bridge = cv_bridge.CvBridge()
        self.__make_dirs()

    def __make_dirs(self):
        self.rgb_save_dir = os.path.join(self.save_dir, 'rgb')
        self.depth_save_dir = os.path.join(self.save_dir, 'depth')
        self.ply_save_dir = os.path.join(self.save_dir, 'ply')
        if not os.path.exists(self.rgb_save_dir):
            os.makedirs(self.rgb_save_dir)
        if not os.path.exists(self.depth_save_dir):
            os.makedirs(self.depth_save_dir)
        if not os.path.exists(self.ply_save_dir):
            os.makedirs(self.ply_save_dir)

    def capture_single_image_and_save(self):
        # Wait for one message from each topic
        rgb_msg = rospy.wait_for_message(self.rgb_topic, Image)
        depth_msg = rospy.wait_for_message(self.depth_topic, Image)
        camera_info_msg = rospy.wait_for_message(self.camera_info_topic, CameraInfo)

        # Convert messages to OpenCV images
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=self.depth_encoding)

        # Handle depth encoding
        if self.depth_encoding == '32FC1':
            depth_image = np.nan_to_num(x=depth_image, nan=0.0)
            depth_image = (depth_image * 1000).astype(np.uint16)

        # Save RGB and depth images
        rgb_path = os.path.join(self.rgb_save_dir, 'captured_rgb.png')
        depth_path = os.path.join(self.depth_save_dir, 'captured_depth.png')
        cv2.imwrite(rgb_path, rgb_image)
        cv2.imwrite(depth_path, depth_image)

        # Convert to PLY
        ply_path = os.path.join(self.ply_save_dir, 'captured.ply')
        rgbd2ply = RVLPYRGBD2PLY.RGBD2PLY()
        rgbd2ply.pixel_array_to_ply(
            rgb_image,
            depth_image,
            camera_info_msg.K[0],  # fx
            camera_info_msg.K[4],  # fy
            camera_info_msg.K[2],  # cx
            camera_info_msg.K[5],  # cy
            camera_info_msg.width,
            camera_info_msg.height,
            0.05,  # Scale factor
            ply_path
        )

        rospy.loginfo(f"Saved RGB image to {rgb_path}")
        rospy.loginfo(f"Saved depth image to {depth_path}")
        rospy.loginfo(f"Saved PLY file to {ply_path}")

        return rgb_path, depth_path, ply_path