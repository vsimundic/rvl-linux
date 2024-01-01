import os
import numpy as np
import cv2

from detectron2.config import get_cfg
from detectron2.engine.defaults import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.engine import default_setup
from tensormask import add_tensormask_config

class Detectron2Tensormask:
    def __init__(self, cfg_args, rgb_save_path, depth_save_path):

        # Setup configuration
        self.model_path = '/home/RVLuser/detectron2/projects/TensorMask/checkpoints/tensormask_R_50_FPN_1x.pkl'
        self.config_path = '/home/RVLuser/detectron2/projects/TensorMask/configs/tensormask_R_50_FPN_1x.yaml'
        self.cfg = self.setup_tensormask(cfg_args)

        # Predictor object
        self.predictor = DefaultPredictor(self.cfg)

        # Save paths
        self.rgb_image_save_path_root = rgb_save_path
        self.depth_image_save_path_root = depth_save_path
        if self.rgb_image_save_path_root:
            self.image_counter = len(os.listdir(self.rgb_image_save_path_root))


    def setup_tensormask(self, args):
        """
        Basic config and setup
        """
        # Load a default config
        cfg = get_cfg()
        add_tensormask_config(cfg)
        cfg.merge_from_file(self.config_path) # Load the TensorMark config file
        cfg.MODEL.WEIGHTS = self.model_path  # Load the TensorMark weights
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # Threshold for detection
        cfg.freeze()
        default_setup(cfg, args)

        return cfg

    def segment_rgb_and_depth_images(self, rgb_image_in, depth_image_in, num_img, visualize_images=True, save_images=True):
        def segment_rgb_image(img_in, human_threshold=0.4, num_img=0, visualize_images=True, save_images=False):

            out = self.predictor(img_in)
            instances = out["instances"] # get all instances from output
            
            # Filter only humans (class 0)
            instances_humans = instances[instances.pred_classes == 0] # binary mask of the human
            instances_humans = instances_humans[instances_humans.scores > human_threshold]

            # Take human mask  
            instance_mask = instances_humans.pred_masks.cpu().numpy()
            instance_mask = np.logical_or.reduce(instance_mask, axis=0)

            v = Visualizer(img_in[:, :, ::-1], instance_mode=1)
            out_drawn_instances = v.draw_instance_predictions(instances_humans.to("cpu"))
            rgb_seg_image = out_drawn_instances.get_image() # [:, :, ::-1]

            if visualize_images:
                cv2.imshow('Segmented RGB image', rgb_seg_image)
                cv2.waitKey(1)

            if save_images:
                cv2.imwrite(os.path.join(self.rgb_image_save_path_root, '%04d.png' % num_img), rgb_seg_image)
                self.image_counter += 1
            
            return rgb_seg_image, instance_mask

        def segment_depth_image(img_in, num_img, instance_mask):
            
            depth_image_to_save = None
            
            if instance_mask.size > 0:
                depth_seg_image = np.where(instance_mask == True, 0., img_in)  # where binary mask is true, put value 0, else the normal depth value
                depth_seg_image = np.reshape(depth_seg_image, [np.shape(img_in)[0], np.shape(img_in)[1]]) 
                depth_seg_image = np.uint16(depth_seg_image)
                depth_image_to_save = depth_seg_image
            else:
                depth_image_to_save = img_in
            
            cv2.imwrite(os.path.join(self.depth_image_save_path_root, '%04d.png' % num_img), depth_image_to_save)
            return depth_image_to_save
        
        rgb_seg_image, instance_mask = segment_rgb_image(rgb_image_in, num_img=num_img, visualize_images=visualize_images, save_images=save_images)
        depth_image_to_save = segment_depth_image(depth_image_in, num_img, instance_mask=instance_mask)
        return rgb_seg_image, depth_image_to_save
