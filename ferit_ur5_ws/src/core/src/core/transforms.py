import numpy as np

def rot_x(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[1, 0, 0],
                  [0, c, -s],
                  [0, s, c]])
    
    return R
    

def rot_y(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, 0, s],
                  [0, 1, 0],
                  [-s, 0, c]])
    return R

def rot_z(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    
    return R

