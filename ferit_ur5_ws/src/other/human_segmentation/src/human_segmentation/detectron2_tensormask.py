import os
import numpy as np
import cv2

from detectron2.config import get_cfg
from detectron2.engine.defaults import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.engine import default_setup
from tensormask import add_tensormask_config

class Detectron2Tensormask:
    def __init__(self, cfg_args, config):
        self.__config = config

        self.__visualize_images = self.__config['visualize_images']
        self.__save_images = self.__config['save_images']
        self.__instance_threshold = self.__config['instance_threshold']

        # Setup configuration
        self.__model_path = config['model_path']
        self.__config_path = config['config_path']
        # self.__model_path = '/home/RVLuser/detectron2/projects/TensorMask/checkpoints/tensormask_R_50_FPN_1x.pkl'
        # self.__config_path = '/home/RVLuser/detectron2/projects/TensorMask/configs/tensormask_R_50_FPN_1x.yaml'
        self.__cfg = self.__setup_tensormask(cfg_args)

        # Predictor object
        self.__predictor = DefaultPredictor(self.__cfg)

        # Save paths
        self.__rgb_seg_folder = self.__config['rgb_seg_folder']
        self.__rgb_folder = self.__config['rgb_folder']
        self.__depth_seg_folder = self.__config['depth_seg_folder']

        # if self.__image_save_path_root:
        #     self.__image_counter = len(os.listdir(os.path.join(self.__image_save_path_root, self.__rgb_seg_folder)))


    def __setup_tensormask(self, args):
        """
        Basic config and setup
        """
        # Load a default config
        cfg = get_cfg()
        add_tensormask_config(cfg)
        cfg.merge_from_file(self.__config_path) # Load the TensorMark config file
        cfg.MODEL.WEIGHTS = self.__model_path  # Load the TensorMark weights
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = self.__config['roi_heads_threshold']  # Threshold for detection
        cfg.freeze()
        default_setup(cfg, args)

        return cfg

    def segment_rgb_and_depth_images(self, rgb_image_in, depth_image_in, image_counter, save_path):
        def segment_rgb_image(img_in, human_threshold=0.4, visualize_images=True, save_images=False):

            out = self.__predictor(img_in)
            instances = out['instances'] # get all instances from output
            
            # Filter only humans (class 0)
            instances_humans = instances[instances.pred_classes == 0] # binary mask of the human
            instances_humans = instances_humans[instances_humans.scores > human_threshold]

            # Take human mask  
            instance_mask = instances_humans.pred_masks.cpu().numpy()
            instance_mask = np.logical_or.reduce(instance_mask, axis=0)

            v = Visualizer(img_in[:, :, ::-1], instance_mode=1)
            out_drawn_instances = v.draw_instance_predictions(instances_humans.to('cpu'))
            rgb_seg_image = out_drawn_instances.get_image() # [:, :, ::-1]

            if visualize_images:
                cv2.imshow('Segmented RGB image', rgb_seg_image)
                cv2.waitKey(1)

            if save_images:
                # cv2.imwrite(os.path.join(self.__image_save_path_root, self.__rgb_seg_folder, '%04d.png' % (self.__image_counter,)), rgb_seg_image)
                # self.__image_counter += 1
                cv2.imwrite(os.path.join(save_path, self.__rgb_seg_folder, '%04d.png' % (image_counter)), rgb_seg_image)

                img_in = cv2.cvtColor(img_in, cv2.COLOR_RGB2BGR)
                cv2.imwrite(os.path.join(save_path, self.__rgb_folder, '%04d.png' % (image_counter)), img_in)
            
            return instance_mask

        def segment_depth_image(img_in, instance_mask):
            
            depth_image_to_save = None
            
            if instance_mask.size > 0:
                depth_seg_image = np.where(instance_mask == True, 0., img_in)  # where binary mask is true, put value 0, else the normal depth value
                depth_seg_image = np.reshape(depth_seg_image, [np.shape(img_in)[0], np.shape(img_in)[1]])
                depth_image_to_save = depth_seg_image
            else:
                depth_image_to_save = img_in

            cv2.imwrite(os.path.join(save_path, self.__depth_seg_folder, '%04d.png' % (image_counter,)), depth_image_to_save.astype(np.uint16))


        instance_mask = segment_rgb_image(rgb_image_in, visualize_images=self.__visualize_images, save_images=self.__save_images, human_threshold=self.__instance_threshold)
        segment_depth_image(depth_image_in, instance_mask=instance_mask)
