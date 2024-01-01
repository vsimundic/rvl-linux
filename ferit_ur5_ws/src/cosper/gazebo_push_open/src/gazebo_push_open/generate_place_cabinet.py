#!/usr/bin/python

import rospy
import os

from core.read_config import read_config
from cosper.gazebo_push_open.src.cabinet_model import generate_cabinet_urdf_from_door_panel

rospy.init_node('node_generate_place_cabinet')

while not rospy.is_shutdown():
    pass

def generate_cabinet(width, height, depth):
    pass