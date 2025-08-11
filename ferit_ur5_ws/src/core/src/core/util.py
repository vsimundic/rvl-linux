import yaml
import rospkg
import os
import pandas as pd
from pandas.errors import EmptyDataError
import numpy as np

def read_config(cfg_filename: str):
    rp = rospkg.RosPack()
    # demo_pkg_path = rp.get_path('a_demo')
    # cfg_filepath = os.path.join(demo_pkg_path, 'config', cfg_filename)
    
    if not os.path.isfile(cfg_filename):
        raise Exception('The file does not exist.')
    if not cfg_filename.endswith('.yaml'):
        raise Exception('A config file must end with .yaml.')
    
    try:
        with open(cfg_filename, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
    except:
        raise Exception('Config must be a valid YAML file.')
    
    return config

def read_csv_DataFrame(path:str, separator:str=',') -> pd.DataFrame:
    try:
        df = pd.read_csv(filepath_or_buffer=path, sep=separator, header=0)
    except EmptyDataError as e:
        return None
    return df

def sample_unit_sphere_angles(n_samples):
    """
    Alternative method using spherical coordinates.
    """
    np.random.seed(0)  
    theta = np.random.uniform(0, 2*np.pi, n_samples)  # azimuth
    phi = np.arccos(np.random.uniform(-1, 1, n_samples))  # inclination
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)
    return np.stack([x, y, z], axis=1)

def angular_difference(q1, q2):
    return np.arctan2(np.sin(q1 - q2), np.cos(q1 - q2))

def get_nearest_joints(candidates, current):
    diffs = angular_difference(candidates, current)
    dists = np.linalg.norm(diffs, axis=1)
    closest_index = np.argmin(dists)
    return candidates[closest_index], closest_index

def get_nearest_joints_with_dist(candidates, current):
    diffs = angular_difference(candidates, current)
    dists = np.linalg.norm(diffs, axis=1)
    closest_index = np.argmin(dists)
    return candidates[closest_index], closest_index, dists[closest_index]

def get_nearest_joints_pair_indices(joint_array1, joint_array2):
    """
    Find the indices of the closest pairs of joints between two arrays.
    
    Args:
        joint_array1 (np.ndarray): First array of joint angles.
        joint_array2 (np.ndarray): Second array of joint angles.
        
    Returns:
        np.ndarray: Indices of the closest pairs of joints.
    """
    indices = []
    dists = []
    for i, joint1 in enumerate(joint_array1):
        _, index, dist = get_nearest_joints_with_dist(joint_array2, joint1)
        indices.append((i, index))
        dists.append(dist)
    idx = np.argmin(dists)
    return indices[idx]


def furthest_point_sampling(points, num_samples):
    N = points.shape[0]
    sampled_indices = []
    distances = np.full(N, np.inf)

    idx = np.random.randint(0, N)
    sampled_indices.append(idx)

    for _ in range(1, num_samples):
        current_point = points[idx]
        dist = np.linalg.norm(points - current_point, axis=1)
        distances = np.minimum(distances, dist)

        idx = np.argmax(distances)
        sampled_indices.append(idx)

    return sampled_indices
