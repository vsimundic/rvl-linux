#!/usr/bin/python

from xml.dom import minidom
import xml.etree.ElementTree as gfg
import os
from math import pi
import numpy as np


def generate_cabinet_urdf_from_door_panel(w_door, h_door, d_door, save_path):
    static_d = 0.3
    moving_to_static_part_distance = 0.005
    axis_distance = 0.01
    static_side_width = 0.018
    w = w_door + 2*(moving_to_static_part_distance + static_side_width)
    h = h_door + 2*(moving_to_static_part_distance + static_side_width)
    d = d_door
    base_link_name = 'base_cabinet_link'
    door_panel_link_name = 'door_link'

    # final visual is a union of panels (no need make some panels shorter)
    panel_dims = np.array([[static_d, w, d], # panel bottom
                        [static_d, d, h], # side
                        [static_d, d, h], # side
                        [static_d, w, d], # top
                        [d, w, h]]) # back

    # with respect to the center of the cuboid
    panel_positions = np.array([[0., 0., -(h/2. - d/2.)], # bottom panel
                    [0., w/2. - d/2., 0.], # side panel
                    [0., -(w/2. - d/2.), 0.], # side panel
                    [0., 0., h/2. - d/2.], # top
                    [-(static_d/2.-d/2.), 0., 0.]]) # back panel


    root = gfg.Element('robot')
    root.set('name', 'my_cabinet')

    world_link = gfg.SubElement(root, 'link')
    world_link.set('name', 'world')

    # Virtual joint
    virtual_joint = gfg.SubElement(root, 'joint')
    virtual_joint.set('name', 'virtual_joint')
    virtual_joint.set('type', 'fixed')
    vj_parent = gfg.SubElement(virtual_joint, 'parent')
    vj_parent.set('link', 'world')
    vj_child = gfg.SubElement(virtual_joint, 'child')
    vj_child.set('link', base_link_name)
    vj_origin = gfg.SubElement(virtual_joint, 'origin')
    vj_origin.set('xyz', '0.0 0.0 0.0')
    vj_origin.set('rpy', '0.0 0.0 0.0')

    base_door_link = gfg.SubElement(root, 'link')
    base_door_link.set('name', base_link_name)

    # Cabinet panels
    for i in range(panel_positions.shape[0]):
        panel_visual = gfg.SubElement(base_door_link, 'visual')

        pl_visual_origin = gfg.SubElement(panel_visual, 'origin')
        pl_visual_origin.set('xyz', '{} {} {}'.format(panel_positions[i, 0], panel_positions[i, 1], panel_positions[i, 2]))
        pl_visual_origin.set('rpy', '0.0 0.0 0.0')

        pl_visual_geom = gfg.SubElement(panel_visual, 'geometry')
        pl_visual_geom_box = gfg.SubElement(pl_visual_geom, 'box')
        pl_visual_geom_box.set('size', '{} {} {}'.format(panel_dims[i, 0], panel_dims[i, 1], panel_dims[i, 2]))
        
        panel_collision = gfg.SubElement(base_door_link, 'collision')
        pl_collision_origin = gfg.SubElement(panel_collision, 'origin')
        pl_collision_origin.set('xyz', '{} {} {}'.format(panel_positions[i, 0], panel_positions[i, 1], panel_positions[i, 2]))
        pl_collision_origin.set('rpy', '0.0 0.0 0.0')

        pl_collision_geom = gfg.SubElement(panel_collision, 'geometry')
        pl_collision_geom_box = gfg.SubElement(pl_collision_geom, 'box')
        pl_collision_geom_box.set('size', '{} {} {}'.format(panel_dims[i, 0], panel_dims[i, 1], panel_dims[i, 2]))

    # Moment of inertia for the entire cabinet (without doors)
    pl_inertial = gfg.SubElement(base_door_link, 'inertial')
    m = 0.2
    ixx = 1/12.*m*(w**2 + h**2)
    iyy = 1/12.*m*(static_d**2 + h**2)
    izz = 1/12.*m*(static_d**2 + w**2)
    pl_inertial_mass = gfg.SubElement(pl_inertial, 'mass')
    pl_inertial_mass.set('value', '%f' % m)
    pl_inertial_inertia = gfg.SubElement(pl_inertial, 'inertia')
    pl_inertial_inertia.set('ixx', '%f' % ixx)
    pl_inertial_inertia.set('ixy', '0.0')
    pl_inertial_inertia.set('ixz', '0.0')
    pl_inertial_inertia.set('iyy', '%f' % iyy)
    pl_inertial_inertia.set('iyz', '0.0')
    pl_inertial_inertia.set('izz', '%f' % izz)


    ## Door panel
    # Dimensions
    # door_panel_dims = [d, w - 2*(moving_to_static_part_distance + static_side_width), h - 2 * (moving_to_static_part_distance + static_side_width)]
    door_panel_dims = [d_door, w_door, h_door]
    door_panel_position = [static_d/2. - d/2., 0., 0.]

    door_panel_link = gfg.SubElement(root, 'link')
    door_panel_link.set('name', 'door_link')

    door_panel_visual = gfg.SubElement(door_panel_link, 'visual')
    dpl_visual_origin = gfg.SubElement(door_panel_visual, 'origin')
    dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(door_panel_dims[1]/2. - axis_distance))
    dpl_visual_geom = gfg.SubElement(door_panel_visual, 'geometry')
    dpl_visual_geom_box = gfg.SubElement(dpl_visual_geom, 'box')
    dpl_visual_geom_box.set('size', '{} {} {}'.format(door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))

    door_panel_collision = gfg.SubElement(door_panel_link, 'collision')
    dpl_collision_origin = gfg.SubElement(door_panel_collision, 'origin')
    dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(door_panel_dims[1]/2. - axis_distance))
    dpl_collision_geom = gfg.SubElement(door_panel_collision, 'geometry')
    dpl_collision_geom_box = gfg.SubElement(dpl_collision_geom, 'box')
    dpl_collision_geom_box.set('size', '{} {} {}'.format(door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))

    # Moment of inertia for the doors
    dpl_inertial = gfg.SubElement(door_panel_link, 'inertial')
    m = 0.2
    ixx = 1/12.*m*(door_panel_dims[1]**2 + door_panel_dims[2]**2)
    iyy = 1/12.*m*(door_panel_dims[0]**2 + door_panel_dims[2]**2)
    izz = 1/12.*m*(door_panel_dims[0]**2 + door_panel_dims[1]**2)
    dpl_inertial_mass = gfg.SubElement(dpl_inertial, 'mass')
    dpl_inertial_mass.set('value', '%f' % m)
    dpl_inertial_inertia = gfg.SubElement(dpl_inertial, 'inertia')
    dpl_inertial_inertia.set('ixx', '%f' % ixx)
    dpl_inertial_inertia.set('ixy', '0.0')
    dpl_inertial_inertia.set('ixz', '0.0')
    dpl_inertial_inertia.set('iyy', '%f' % iyy)
    dpl_inertial_inertia.set('iyz', '0.0')
    dpl_inertial_inertia.set('izz', '%f' % izz)

    joint_0 = gfg.SubElement(root, 'joint')
    joint_0.set('name', 'joint_0')
    joint_0.set('type', 'revolute')

    j0_origin = gfg.SubElement(joint_0, 'origin')
    j0_origin.set('xyz', '{} {} 0'.format(static_d/2 - d/2, -door_panel_dims[1]/2. + axis_distance))
    j0_axis = gfg.SubElement(joint_0, 'axis')
    j0_axis.set('xyz', '0 0 1')
    j0_parent = gfg.SubElement(joint_0, 'parent')
    j0_parent.set('link', base_link_name)
    j0_child = gfg.SubElement(joint_0, 'child')
    j0_child.set('link', door_panel_link_name)
    j0_limit = gfg.SubElement(joint_0, 'limit')
    j0_limit.set('effort', '100')
    j0_limit.set('lower', '{}'.format(-pi/2.))
    j0_limit.set('upper', '0')
    j0_limit.set('velocity', '50')
    j0_dynamics = gfg.SubElement(joint_0, 'dynamics')
    j0_dynamics.set('friction', '1.5')

    gazebo_ = gfg.SubElement(root, 'gazebo')
    gazebo_.set('reference', 'my_cabinet')
    gazebo_material = gfg.SubElement(gazebo_, 'material')
    gazebo_material.text = 'Gazebo/Blue'

    # Prettify for writing in a file
    xmlstr = minidom.parseString(gfg.tostring(root)).toprettyxml(indent='\t')

    with open (save_path, 'w') as f:
        f.write(xmlstr)

    
if __name__ == '__main__':
    generate_cabinet_urdf_from_door_panel(0.3, 0.5, 0.018, '/home/RVLuser/ur5_ws/src/dd_man/cabinet_models/my_cabinet/my_cabinet_test.urdf')