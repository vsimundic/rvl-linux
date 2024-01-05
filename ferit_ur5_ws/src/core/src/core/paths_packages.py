import rospkg
import os

ROSPACK = rospkg.RosPack()
# WORKSPACE_PATH = ROSPACK.get_path('')

def get_package_name_from_node(node_name: str):
    # The node name is in the form /package_name/node_name
    return node_name.split('/')[1]

def get_package_path_from_name(package_name: str):
    return ROSPACK.get_path(package_name)