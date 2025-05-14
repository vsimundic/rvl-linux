import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_matrix
import rospy
import tf

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

def matrix_to_pose(T: np.ndarray):
    q = quaternion_from_matrix(T) # xyzw

    pose = Pose()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    pose.position.x = T[0, 3]
    pose.position.y = T[1, 3]
    pose.position.z = T[2, 3]
    return pose


def matrix_to_pose_stamped(T: np.ndarray, frame_id: str):
    q = quaternion_from_matrix(T) # xyzw

    pose_s = PoseStamped()
    pose_ = Pose()
    pose_s.header.frame_id = frame_id
    pose_s.header.stamp = rospy.Time.now()

    pose_.orientation.x = q[0]
    pose_.orientation.y = q[1]
    pose_.orientation.z = q[2]
    pose_.orientation.w = q[3]
    pose_.position.x = T[0, 3]
    pose_.position.y = T[1, 3]
    pose_.position.z = T[2, 3]
    pose_s.pose = pose_
    return pose_s


def pose_stamped_to_matrix(pose: PoseStamped):
    pose_ = pose.pose
    q_ = [pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w]
    t_ = np.array([pose_.position.x, pose_.position.y, pose_.position.z])
    T = quaternion_matrix(q_)
    T[:3, 3] = np.array(t_)
    return T

def pose_to_matrix(pose: Pose):
    pose_ = pose
    q_ = [pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w]
    t_ = np.array([pose_.position.x, pose_.position.y, pose_.position.z])
    T = quaternion_matrix(q_)
    T[:3, 3] = np.array(t_)
    return T

def matrix_to_euler(T: np.ndarray):
    """
    Converts a 4x4 transformation matrix to Euler angles (roll, pitch, yaw).
    
    Args:
        matrix (list or numpy.ndarray): 4x4 rotation matrix.
        axes (str): The axes sequence for Euler angles (default is 'sxyz').
    
    Returns:
        tuple: (roll, pitch, yaw) in radians.
    
    Raises:
        ValueError: If the input matrix is not 4x4.
    """
    if T.shape[0] != 4 or T.shape[1] != 4:
        raise  ValueError("Input must be a 4x4 matrix.")
    
    return euler_from_matrix(T, axes='sxyz')


def get_frame_transform(parent_frame: str, child_frame: str):
    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            trans, rot = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            break   
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # return None
            continue
    
    T = quaternion_matrix(rot)
    T[:3, 3] = trans.copy()

    return T

def rodrigues_rotate_vector(dir_init, axis, angle_deg):
    axis = axis / np.linalg.norm(axis)  # normalize axis of rotation
    angle_rad = np.deg2rad(angle_deg)

    cos_theta = np.cos(angle_rad)
    sin_theta = np.sin(angle_rad)

    cross = np.cross(axis, dir_init) # cross product to get the perpendicular vector
    dot = np.dot(axis, dir_init) # dot product to get the projection of dir_init on axis
    # Rodrigues' rotation formula
    rotated = dir_init * cos_theta + cross * sin_theta + axis * dot * (1 - cos_theta)
    return rotated
