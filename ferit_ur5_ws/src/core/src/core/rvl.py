import numpy as np
import RVLPYRGBD2PLY as rvl_rgbd2ply
import PIL.Image
import os
import yaml

class RVLRGBD2PLY():
    def __init__(self, root_path, rgb_dirname, depth_dirname, ply_dirname, camera_params_filename):
        self.__rgbd2ply = rvl_rgbd2ply.RGBD2PLY()
        self.root_path = root_path
        self.scene_sequence_path = os.path.join(self.root_path, 'sceneSequence.txt')
        self.rgb_dirpath = os.path.join(self.root_path, rgb_dirname)
        self.depth_dirpath = os.path.join(self.root_path, depth_dirname)
        self.ply_dirpath = os.path.join(self.root_path, ply_dirname)

        self.camera_params_path = os.path.join(self.root_path, camera_params_filename)
        self.camera_matrix = ''
        self.image_width = -1
        self.image_height = -1

        self.ply_paths = []
        self.rgb_paths = []
        self.depth_paths = []

        self.__load_paths()
        self.__load_camera_params()
        
    def __load_camera_params(self):
        with open(self.camera_params_path, 'r') as f:
            try:
                cam_params = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)
            
            self.image_width = cam_params['width']
            self.image_height = cam_params['height']
            
            self.camera_matrix = np.array(cam_params['K']).reshape((3,3))

    def __load_paths(self):
        lines =''
        with open(self.scene_sequence_path, 'r') as f:
            lines = f.readlines()
        
        for line in lines:
            if line.strip() == 'end':
                break
            rgb_img_path = os.path.join(self.rgb_dirpath, line.strip())
            self.rgb_paths.append(rgb_img_path)
            depth_img_path = os.path.join(self.depth_dirpath, line.strip())
            self.depth_paths.append(depth_img_path)
            ply_path = os.path.join(self.ply_dirpath, line.strip().split('.')[0] + '.ply')
            self.ply_paths.append(ply_path)
    
    def save_plys(self):
        for i in range(len(self.rgb_paths)):
            rgb_img_path = self.rgb_paths[i]
            depth_img_path = self.depth_paths[i]
            ply_path = self.ply_paths[i]
            self.ply_paths.append(ply_path)

            rgb_img = PIL.Image.open(rgb_img_path)
            rgb_array = np.array(rgb_img).astype(np.byte).copy()
            bgr_array = np.stack((rgb_array[:,2], rgb_array[:,1], rgb_array[:,0]), axis=1)
            depth_img = PIL.Image.open(depth_img_path)
            depth_array = np.array(depth_img).astype(np.short).copy()
            print(ply_path)
            self.__rgbd2ply.pixel_array_to_ply(bgr_array, depth_array,
                                               self.camera_matrix[0, 0],
                                               self.camera_matrix[1, 1],
                                               self.camera_matrix[0, 2],
                                               self.camera_matrix[1, 2],
                                               self.image_width, 
                                               self.image_height,
                                               0.050,
                                               ply_path)
