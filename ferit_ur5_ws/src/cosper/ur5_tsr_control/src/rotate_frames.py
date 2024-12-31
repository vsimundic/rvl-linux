#!/usr/bin/env python

from core.transforms  import matrix_to_pose, rot_y, rot_z
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
import numpy as np
import numpy as np

T_G_W = np.eye(4)
T_G_W[:3,:3] = np.array([[0, 0,  1],
                         [-1, 0, 0],
                         [0, -1, 0]])

Tz = np.eye(4)
Tz[:3,:3] =  rot_z(-np.pi/2.)
Tz_45 = np.eye(4)
Tz_45[:3,:3] =  rot_z(np.pi/4.)
Ty = np.eye(4)
Ty[:3,:3] =  rot_y(np.pi/2.)
T_G_W_manual = Ty @ Tz @ Tz_45

quat = quaternion_from_matrix(T_G_W_manual)
print(quat)