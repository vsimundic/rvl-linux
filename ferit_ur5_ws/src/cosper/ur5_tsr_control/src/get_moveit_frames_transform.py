#!/usr/bin/env python

import rospy
import tf
import numpy as np
from core.transforms import matrix_to_pose, rot_z

T_w_e = np.eye(4)
R_t_g = np.array([[0,0,1],
                  [-1,0,0],
                  [0,-1,0]
])
T_w_e[:3,:3] = R_t_g

T_e_erot = np.eye(4)
T_e_erot[:3,:3] = rot_z(np.deg2rad(45))

Rz = rot_z(np.deg2rad(45.0))
R_t_g_rotated = np.eye(4)
R_t_g_rotated[:3, :3] = R_t_g 
# R_t_g_rotated[:3, :3] = R_t_g @ Rz
# R_t_g_rotated = R_t_g @ Rz
print(R_t_g_rotated)
print(matrix_to_pose(R_t_g_rotated))

def print_transform(parent_frame, child_frame):
    rospy.init_node('frame_transform_printer')

    listener = tf.TransformListener()

    rate = rospy.Rate(1)  # Set the rate to 1 Hz, adjust as needed

    while not rospy.is_shutdown():
        try:
            # Get the latest transform between the frames
            (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

            # Print translation and rotation
            print(f"Translation from {parent_frame} to {child_frame}:")
            print(f"  x: {trans[0]}")
            print(f"  y: {trans[1]}")
            print(f"  z: {trans[2]}")

            print(f"Rotation from {parent_frame} to {child_frame} (quaternion):")
            print(f"  x: {rot[0]}")
            print(f"  y: {rot[1]}")
            print(f"  z: {rot[2]}")
            print(f"  w: {rot[3]}")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logwarn(f"Could not get transform from {parent_frame} to {child_frame}: {ex}")

        rate.sleep()

if __name__ == '__main__':
    try:
        # Replace 'base_link' and 'tool0' with your desired frames
        print_transform('world', 'table')
    except rospy.ROSInterruptException:
        pass