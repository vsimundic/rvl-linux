import numpy as np
import RVLPYRGBD2PLY as rvl
from PIL import Image
import os.path

print('starting...')

rgbd2ply = rvl.RGBD2PLY()
image_directory_name = '/home/RVLuser/rvl-linux/data/femto_test_cabinet'
rgb_directory_name = 'rgb'
depth_directory_name = 'depth_seg'
ply_directory_name = 'PLY_seg'
image_name = '0000'
rgb_png_file_name = os.path.join(image_directory_name, rgb_directory_name, image_name + '.png')
depth_png_file_name = os.path.join(image_directory_name, depth_directory_name, image_name + '.png')
ply_file_name = os.path.join(image_directory_name, ply_directory_name, image_name + '.ply')

# Create PLY directly from PNG files.

#rgbd2ply.png_to_ply(rgb_png_file_name, depth_png_file_name,
#597.9033203125,
#598.47998046875,
#323.8436584472656,
#236.32774353027344,
#640,
#480,
#0.050,
#ply_file_name)

# Create PLY from numpy arrays.

rgb_img = Image.open(rgb_png_file_name)
rgb_array = np.array(rgb_img.getdata()).astype(np.byte).copy()
bgr_array = np.stack((rgb_array[:,2], rgb_array[:,1], rgb_array[:,0]), axis=1)
depth_img = Image.open(depth_png_file_name)
depth_array = np.array(depth_img.getdata()).astype(np.short).copy()
rgbd2ply.pixel_array_to_ply(bgr_array, depth_array, 
518.61083984375,
518.61083984375,
308.2198486328125,
238.864990234375,
640,
480,
0.050,
ply_file_name)

print('completed.')
