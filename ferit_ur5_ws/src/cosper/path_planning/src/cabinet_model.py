#!/usr/bin/python

import rospy
from xml.dom import minidom
import xml.etree.ElementTree as gfg
import os
from math import pi
import numpy as np
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel, GetModelState, GetJointProperties, GetJointPropertiesRequest
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix
import open3d as o3d
# from core.transforms import rot_z

def rot_z(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, -s, 0],
                  [s, c, 0],
                  [0, 0, 1]])
    return R

def rot_y(angle_rad):
    s = np.sin(angle_rad)
    c = np.cos(angle_rad)
    R = np.array([[c, 0, s],
                  [0, 1, 0],
                  [-s, 0, c]])
    return R


class Cabinet():
    # def __init__(self, w_door: float, h_door: float, d_door: float=0.018, static_d: float=0.3, axis_pos: int=1, position: np.array=np.array([0, 0, 0]), angle_deg:float=0, save_path:str=None):
    # def __init__(self, door_params:np.array=np.array([0.1, 0.1, 0.018, 0.1]), axis_pos: int=1, position: np.array=np.array([0, 0, 0]), rotz_deg:float=0, save_path:str=None):
    def __init__(self, door_params:np.array=np.array([0.1, 0.1, 0.018, 0.1]), axis_pos: int=1, T_A_S: np.ndarray=np.eye(4), save_path:str=None):
        # S = World (scene)
        # O = centroid of cabinet
        # F = top left corner of static part of cabinet
        # A = Door axis
        # D = inner left upper point on door panel

        self.w_door = door_params[0]
        self.h_door = door_params[1]
        self.d_door = door_params[2]
        self.static_d = door_params[3]
        self.moving_to_static_part_distance = 0.005
        self.axis_distance = 0.01
        self.static_side_width = 0.018
        self.base_link_name = 'base_cabinet_link'
        self.door_panel_link_name = 'door_link'
        self.door_joint_name = 'door_joint'
        self.cabinet_name = 'my_cabinet'
        self.save_path = save_path
        # self.position = position
        # self.rotz_deg = rotz_deg
        self.T_A_S = T_A_S
        self.current_angle_state_deg = 0.
        if not axis_pos == 1 and not axis_pos == -1:
            self.axis_pos = 1
        else:
            self.axis_pos = axis_pos

        self.xml_model = None
        if self.save_path is not None:
            self.xml_model = self.generate_cabinet_urdf_from_door_panel()

        self.setup_matrices()

        self.mesh = self.create_mesh()


    def setup_matrices(self):
        # Top left corner (S-frame) w.r.t. centroid of cabinet
        self.T_F_O = np.eye(4)
        self.T_F_O[:3, :3] = np.array([[0, 0, -1],
                                [1, 0, 0],
                                [0, -1, 0]])
        self.T_F_O[:3, 3] = np.array([self.static_d/2.,
                               -(self.w_door/2. + self.moving_to_static_part_distance + self.d_door),
                                self.h_door/2. + self.moving_to_static_part_distance + self.d_door])

        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.pi)

        self.T_A_O = np.eye(4)
        if self.axis_pos < 0:
            self.T_A_O = Tz @ self.T_A_O
        self.T_A_O[:3, 3] = np.array([self.static_d/2. - self.d_door/2., self.axis_pos*(self.w_door/2. - self.axis_distance), 0.])
        self.T_A_O_init = self.T_A_O.copy()

        # Cabinet centroid to world

        # self.T_O_S = self.__get_world_pose(self.position[0], self.position[1], self.position[2], spawn_angle_deg=self.rotz_deg)
        self.T_O_S = self.T_A_S @ np.linalg.inv(self.T_A_O_init)

        # Panel point to axis
        self.T_D_A = np.eye(4)
        # self.T_D_A[:3, :3] = np.array([[0, 0, -1],
        #                               [1, 0, 0],
        #                               [0, -1, 0]])
        self.T_D_A[:3, :3] = rot_y(np.radians(-self.axis_pos*90.)) @ rot_z(np.radians(self.axis_pos*90.))

        self.T_D_A[:3, 3] = np.array([-self.axis_pos*(self.d_door/2.),
                                     -(self.w_door - self.axis_distance),
                                     self.h_door/2.])

        # # Door panel centroid to axis
        # self.T_D_A_init = np.eye(4)
        # self.T_D_A_init[:3, 3] = np.array([self.axis_pos*(self.d_door/2.), -(self.w_door/2. - self.axis_distance), self.h_door/2.])

    def generate_cabinet_urdf_from_door_panel(self):
        static_d = self.static_d
        moving_to_static_part_distance = self.moving_to_static_part_distance
        axis_distance = self.axis_distance
        static_side_width = self.static_side_width
        bottom_panel_height = 0.004
        w = self.w_door + 2*(moving_to_static_part_distance + static_side_width)
        h = self.h_door + 2*moving_to_static_part_distance + static_side_width + bottom_panel_height
        d = self.d_door
        base_link_name = self.base_link_name
        door_panel_link_name = self.door_panel_link_name
        axis_pos = self.axis_pos

        save_path = self.save_path

        # final visual is a union of panels (no need make some panels shorter)
        panel_dims = np.array([[static_d, w, bottom_panel_height], # panel bottom
                            [static_d, d, h], # side
                            [static_d, d, h], # side
                            [static_d, w, d], # top
                            [d, w, h]]) # back

        # with respect to the center of the cuboid
        panel_positions = np.array([[0., 0., -self.h_door*0.5 - 0.005 - bottom_panel_height*0.5], # bottom panel
                        [0., w/2. - d/2., bottom_panel_height*0.5+moving_to_static_part_distance], # side panel
                        [0., -(w/2. - d/2.), bottom_panel_height*0.5+moving_to_static_part_distance], # side panel
                        [0., 0., self.h_door*0.5+moving_to_static_part_distance+d*0.5], # top
                        [-(static_d/2.-d/2.), 0., bottom_panel_height*0.5+moving_to_static_part_distance]]) # back panel


        root = gfg.Element('robot')
        root.set('name', self.cabinet_name)

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
            panel_collision.set('name', base_link_name+'_collision_%d' % i)
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
        door_panel_dims = [self.d_door, self.w_door, self.h_door]
        door_panel_position = [static_d/2. - d/2., 0., 0.]

        door_panel_link = gfg.SubElement(root, 'link')
        door_panel_link.set('name', 'door_link')

        door_panel_visual = gfg.SubElement(door_panel_link, 'visual')
        dpl_visual_origin = gfg.SubElement(door_panel_visual, 'origin')
        # dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(-(door_panel_dims[1]/2. - axis_distance)))
        dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(-axis_pos*(door_panel_dims[1]/2. - axis_distance)))
        dpl_visual_geom = gfg.SubElement(door_panel_visual, 'geometry')
        dpl_visual_geom_box = gfg.SubElement(dpl_visual_geom, 'box')
        dpl_visual_geom_box.set('size', '{} {} {}'.format(door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))

        door_panel_collision = gfg.SubElement(door_panel_link, 'collision')
        door_panel_collision.set('name', self.door_panel_link_name+'_collision')
        dpl_collision_origin = gfg.SubElement(door_panel_collision, 'origin')
        dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(-axis_pos*(door_panel_dims[1]/2. - axis_distance)))
        # dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(axis_pos*(door_panel_dims[1]/2. - axis_distance)))
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

        door_joint = gfg.SubElement(root, 'joint')
        door_joint.set('name', self.door_joint_name)
        door_joint.set('type', 'revolute')

        j0_origin = gfg.SubElement(door_joint, 'origin')
        j0_origin.set('xyz', '{} {} 0'.format(static_d/2 - d/2, axis_pos*(door_panel_dims[1]/2. - axis_distance)))
        # j0_origin.set('xyz', '{} {} 0'.format(static_d/2 - d/2, -(-door_panel_dims[1]/2. + axis_distance)))
        # j0_origin.set('rpy', '{} {} {}'.format(0,0, np.radians(180)))
        j0_origin.set('rpy', '{} {} {}'.format(0, 0, 0))
        j0_axis = gfg.SubElement(door_joint, 'axis')
        j0_axis.set('xyz', '0 0 1')
        j0_parent = gfg.SubElement(door_joint, 'parent')
        j0_parent.set('link', base_link_name)
        j0_child = gfg.SubElement(door_joint, 'child')
        j0_child.set('link', door_panel_link_name)
        j0_limit = gfg.SubElement(door_joint, 'limit')
        j0_limit.set('effort', '50')

        # if axis_pos == 1:
        if axis_pos < 0:
            j0_limit.set('lower', '0')
            j0_limit.set('upper', '{}'.format(-pi/2.))
        # elif axis_pos == -1:
        elif axis_pos > 0:
            j0_limit.set('lower', '{}'.format(0))
            j0_limit.set('upper', '{}'.format(pi/2.))

        j0_limit.set('velocity', '10')
        j0_dynamics = gfg.SubElement(door_joint, 'dynamics')
        j0_dynamics.set('friction', '1.0')

        # gazebo_ = gfg.SubElement(root, 'gazebo')
        # gazebo_.set('reference', base_link_name)
        # gazebo_material = gfg.SubElement(gazebo_, 'material')
        # gazebo_material.text = 'Gazebo/Blue'

        # Contact sensor
        gazebo_sensor_ = gfg.SubElement(root, 'gazebo')
        gazebo_sensor_.set('reference', base_link_name)
        gazebo_sensor_sensor = gfg.SubElement(gazebo_sensor_, 'sensor')
        gazebo_sensor_sensor.set('name', 'main_sensor')
        gazebo_sensor_sensor.set('type', 'contact')
        gazebo_sensor_sensor_selfCollide = gfg.SubElement(gazebo_sensor_sensor, 'selfCollide')
        gazebo_sensor_sensor_selfCollide.text = 'true'
        gazebo_sensor_sensor_alwaysOn = gfg.SubElement(gazebo_sensor_sensor, 'alwaysOn')
        gazebo_sensor_sensor_alwaysOn.text = 'true'
        gazebo_sensor_sensor_updateRate = gfg.SubElement(gazebo_sensor_sensor, 'updateRate')
        gazebo_sensor_sensor_updateRate.text = '5.0'
        gazebo_sensor_sensor_material = gfg.SubElement(gazebo_sensor_sensor, 'material')
        gazebo_sensor_sensor_material.text = 'Gazebo/Red'
        gazebo_sensor_sensor_contact = gfg.SubElement(gazebo_sensor_sensor, 'contact')

        gazebo_sensor_sensor_contact_0 = gfg.SubElement(gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_0.text = base_link_name + '_collision_%d' % 0 + '_collision'
        gazebo_sensor_sensor_contact_1 = gfg.SubElement(gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_1.text = base_link_name + '_collision_%d' % 1 + '_collision'
        gazebo_sensor_sensor_contact_2 = gfg.SubElement(gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_2.text = base_link_name + '_collision_%d' % 2 + '_collision'
        gazebo_sensor_sensor_contact_3 = gfg.SubElement(gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_3.text = base_link_name + '_collision_%d' % 3 + '_collision'
        gazebo_sensor_sensor_contact_4 = gfg.SubElement(gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_4.text = base_link_name + '_collision_%d' % 4 + '_collision'

        gazebo_sensor_sensor_plugin = gfg.SubElement(gazebo_sensor_sensor, 'plugin')
        gazebo_sensor_sensor_plugin.set('name', 'gazebo_ros_bumper_controller')
        gazebo_sensor_sensor_plugin.set('filename', 'libgazebo_ros_bumper.so')
        gazebo_sensor_sensor_plugin_bumperTopicName = gfg.SubElement(gazebo_sensor_sensor_plugin, 'bumperTopicName')
        gazebo_sensor_sensor_plugin_bumperTopicName.text = '/contact'
        gazebo_sensor_sensor_plugin_frameName = gfg.SubElement(gazebo_sensor_sensor_plugin, 'frameName')
        gazebo_sensor_sensor_plugin_frameName.text = 'world'


        # Door contact sensor
        gazebo_sensor_door = gfg.SubElement(root, 'gazebo')
        gazebo_sensor_door.set('reference', door_panel_link_name)
        gazebo_sensor_doorsensor = gfg.SubElement(gazebo_sensor_door, 'sensor')
        gazebo_sensor_doorsensor.set('name', 'main_sensor')
        gazebo_sensor_doorsensor.set('type', 'contact')
        gazebo_sensor_doorsensor_selfCollide = gfg.SubElement(gazebo_sensor_doorsensor, 'selfCollide')
        gazebo_sensor_doorsensor_selfCollide.text = 'true'
        gazebo_sensor_doorsensor_alwaysOn = gfg.SubElement(gazebo_sensor_doorsensor, 'alwaysOn')
        gazebo_sensor_doorsensor_alwaysOn.text = 'true'
        gazebo_sensor_doorsensor_updateRate = gfg.SubElement(gazebo_sensor_doorsensor, 'updateRate')
        gazebo_sensor_doorsensor_updateRate.text = '5.0'
        gazebo_sensor_doorsensor_material = gfg.SubElement(gazebo_sensor_doorsensor, 'material')
        gazebo_sensor_doorsensor_material.text = 'Gazebo/Red'
        gazebo_sensor_doorsensor_contact = gfg.SubElement(gazebo_sensor_doorsensor, 'contact')

        gazebo_sensor_doorsensor_contact_0 = gfg.SubElement(gazebo_sensor_doorsensor_contact, 'collision')
        gazebo_sensor_doorsensor_contact_0.text = self.door_panel_link_name + '_collision' + '_collision'
        
        gazebo_sensor_doorsensor_plugin = gfg.SubElement(gazebo_sensor_doorsensor, 'plugin')
        gazebo_sensor_doorsensor_plugin.set('name', 'gazebo_ros_bumper_controller')
        gazebo_sensor_doorsensor_plugin.set('filename', 'libgazebo_ros_bumper.so')
        gazebo_sensor_doorsensor_plugin_bumperTopicName = gfg.SubElement(gazebo_sensor_doorsensor_plugin, 'bumperTopicName')
        gazebo_sensor_doorsensor_plugin_bumperTopicName.text = '/contact_door'
        gazebo_sensor_doorsensor_plugin_frameName = gfg.SubElement(gazebo_sensor_doorsensor_plugin, 'frameName')
        gazebo_sensor_doorsensor_plugin_frameName.text = 'contact_door'

        # Prettify for writing in a file
        xmlstr = minidom.parseString(gfg.tostring(root)).toprettyxml(indent='\t')

        if save_path:
            with open (save_path, 'w') as f:
                f.write(xmlstr)

        return xmlstr


    def __get_world_pose(self, x: float, y: float, z: float, spawn_angle_deg: float):
        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array([x, y, z])

        theta = np.radians(spawn_angle_deg)
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(theta)

        T_O_S = T_A_S @ np.linalg.inv(self.T_A_O) @ Tz

        return T_O_S


    def change_door_angle(self, angle_deg):
        self.current_angle_state_deg = angle_deg
        angle_rad = np.radians(angle_deg)

        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(angle_rad)
        self.T_A_O = self.T_A_O_init @ Tz
        return self.T_A_O


    def spawn_model_gazebo(self):

        if self.xml_model is None:
            return None

        init_pose = Pose()
        init_pose.position.x = self.T_O_S[0, 3]
        init_pose.position.y = self.T_O_S[1, 3]
        init_pose.position.z = self.T_O_S[2, 3]

        q = quaternion_from_matrix(self.T_O_S)

        init_pose.orientation.x = q[0]
        init_pose.orientation.y = q[1]
        init_pose.orientation.z = q[2]
        init_pose.orientation.w = q[3]

        spawn_model_msg = SpawnModel()
        spawn_model_msg.model_name = self.cabinet_name
        spawn_model_msg.model_xml = self.xml_model
        spawn_model_msg.robot_namespace = '/'
        spawn_model_msg.initial_pose = init_pose

        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            # _ = spawn_urdf_model_proxy(spawn_model_msg)
            _ = spawn_urdf_model_proxy(self.cabinet_name, self.xml_model, '/', init_pose, '')
            rospy.loginfo('URDF model spawned successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to spawn URDF model: %s' % e)

        rospy.sleep(1)
        success, _ = self.get_model_state_gazebo()

        if not success:
            self.spawn_model_gazebo()

    def get_model_state_gazebo(self):
        rospy.wait_for_service('/gazebo/get_model_state')

        try:
            get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            # _, pose, _, success, status_msg = get_model_state_proxy(self.cabinet_name, 'world')
            res = get_model_state_proxy(self.cabinet_name, 'world')
            # print(status_msg)
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get URDF model: %s' % e)
            return False, None

        # return success, pose
        return res.success, res.pose


    def delete_model_gazebo(self):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            _ = delete_model_proxy(self.cabinet_name)
            rospy.loginfo('URDF deleted successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to delete model: %s' % e)


    def set_door_state_gazebo(self, joint_value_deg):

        joint_value_rad = np.radians(joint_value_deg)

        rospy.wait_for_service('/gazebo/set_model_configuration')

        try:
            set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

            req = SetModelConfigurationRequest()
            req.model_name = self.cabinet_name
            req.urdf_param_name = 'robot_description'  # Assuming URDF is loaded with the parameter name 'robot_description'
            req.joint_names = [self.door_joint_name]
            req.joint_positions = [joint_value_rad]

            _ = set_model_configuration(req)
            rospy.loginfo(f'Joint {self.door_joint_name} set to {joint_value_rad} for model {self.cabinet_name}')
        except rospy.ServiceException as e:
            rospy.logerr(f'Failed to set joint configuration: {e}')


    def get_door_state_gazebo(self):
        rospy.wait_for_service('/gazebo/get_joint_properties')

        try:
            get_door_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
            # type, damping, position, rate, success, status_msg = get_door_state(self.cabinet_name, 'world')
            res = get_door_state(self.door_joint_name)
            print(res.status_message)
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get URDF model: %s' % e)
            return False, None

        return res.success, res.position[0]



    # TEST METHOD
    def spawn_model_gazebo_sphere(self):

        # TEST SPHERE
        with open('/home/RVLuser/ferit_ur5_ws/sphere.urdf', 'r') as f:
            urdf = f.read()

        model_name = 'sphere'
        pose_ = self.__matrix_to_pose(self.get_door_panel_RF_wrt_world())
        reference_frame = 'world'

        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            # _ = spawn_urdf_model_proxy(spawn_model_msg)
            _ = spawn_urdf_model_proxy(model_name, urdf, '/', pose_, '')
            rospy.loginfo('URDF model spawned successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to spawn URDF model: %s' % e)

        # TEST SPHERE 2
        with open('/home/RVLuser/ferit_ur5_ws/sphere.urdf', 'r') as f:
            urdf = f.read()

        model_name = 'sphere2'
        pose_ = self.__matrix_to_pose(self.T_O_S @ self.T_F_O)
        reference_frame = 'world'

        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            # _ = spawn_urdf_model_proxy(spawn_model_msg)
            _ = spawn_urdf_model_proxy(model_name, urdf, '/', pose_, '')
            rospy.loginfo('URDF model spawned successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to spawn URDF model: %s' % e)
    # TEST METHOD
    def delete_model_gazebo_sphere(self):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            _ = spawn_urdf_model_proxy('sphere')
            rospy.loginfo('URDF deleted successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to delete model: %s' % e)

        rospy.wait_for_service('/gazebo/delete_model')
        try:
            spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            _ = spawn_urdf_model_proxy('sphere2')
            rospy.loginfo('URDF deleted successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to delete model: %s' % e)



    # def get_gripper_pose_from_feasible_pose(self, T_F_D: np.ndarray, TX0: np.ndarray, TB0: np.ndarray):
    def get_feasible_pose_wrt_world(self, T_F_D: np.ndarray):
        return self.T_O_S @ self.T_A_O @ self.T_D_A @ T_F_D


    def get_door_panel_RF_wrt_world(self):
        return self.T_O_S @ self.T_A_O @ self.T_D_A


    def create_mesh(self):
        self.dd_static_top_mesh = o3d.geometry.TriangleMesh.create_box(width=self.static_d,
                                                                  height=self.w_door + 2*(self.moving_to_static_part_distance + self.d_door),
                                                                  depth=self.d_door)
        self.dd_static_top_mesh.translate((-self.static_d/2.,
                                         -(self.w_door/2. + self.moving_to_static_part_distance + self.d_door),
                                         (self.h_door/2. + self.moving_to_static_part_distance)))


        self.dd_static_bottom_mesh = o3d.geometry.TriangleMesh.create_box(width=self.static_d,
                                                                     height=self.w_door + 2*(self.moving_to_static_part_distance + self.d_door),
                                                                     depth=self.d_door)
        self.dd_static_bottom_mesh.translate((-self.static_d/2.,
                                         -(self.w_door/2. + self.moving_to_static_part_distance + self.d_door),
                                         -(self.h_door/2. + self.moving_to_static_part_distance + self.d_door)))


        self.dd_static_left_mesh = o3d.geometry.TriangleMesh.create_box(width=self.static_d,
                                                                   height=self.d_door,
                                                                   depth=(self.h_door + 2*self.moving_to_static_part_distance + 2*self.d_door))
        self.dd_static_left_mesh.translate((-self.static_d/2.,
                                       -(self.w_door/2. + self.moving_to_static_part_distance + self.d_door),
                                       -(self.h_door/2. + self.moving_to_static_part_distance + self.d_door)))


        self.dd_static_right_mesh = o3d.geometry.TriangleMesh.create_box(width=self.static_d,
                                                                   height=self.d_door,
                                                                   depth=(self.h_door + 2*self.moving_to_static_part_distance + 2*self.d_door))
        self.dd_static_right_mesh.translate((-self.static_d/2.,
                                       (self.w_door/2. + self.moving_to_static_part_distance),
                                       -(self.h_door/2. + self.moving_to_static_part_distance + self.d_door)))

        self.dd_static_mesh = self.dd_static_top_mesh + self.dd_static_bottom_mesh + self.dd_static_left_mesh + self.dd_static_right_mesh
        self.dd_static_mesh.paint_uniform_color([0.4, 0.4, 0.4])
        self.dd_static_mesh.compute_vertex_normals()

        self.dd_plate_mesh = o3d.geometry.TriangleMesh.create_box(width=self.w_door,
                                                             height=self.h_door,
                                                             depth=self.d_door)

        if self.axis_pos == 1:
            self.dd_plate_mesh.translate((0.0, 0.0, -self.d_door))
        else:
            self.dd_plate_mesh.translate((-self.w_door, 0.0, -self.d_door))

        self.dd_plate_mesh.transform(self.T_A_O @ self.T_D_A)
        self.dd_plate_mesh.paint_uniform_color([0.7, 0.7, 0.7])
        self.dd_plate_mesh.compute_vertex_normals()

        dd_mesh = self.dd_static_mesh + self.dd_plate_mesh
        # dd_mesh = self.dd_static_mesh

        return dd_mesh


    def save_mesh(self, filename):
        T_A_O = self.T_A_O.copy()
        self.T_A_O = self.T_A_O_init
        self.update_mesh()
        o3d.io.write_triangle_mesh(filename, self.mesh)

        self.T_A_O = T_A_O
        self.update_mesh()

    def save_mesh_without_doors(self, filename):
        o3d.io.write_triangle_mesh(filename, self.dd_static_mesh)

    def update_mesh(self):
        self.mesh = self.create_mesh()


    def visualize(self, T_G_S: np.ndarray=None):

        geometries = [self.mesh]

        self.mesh.transform(self.T_O_S)

        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # origin_frame.paint_uniform_color([1, 0, 0])
        geometries.append(origin_frame)


        # left_cf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # left_cf.transform(self.T_O_S)
        # left_cf.translate((-self.static_d/2.,
        #                 -(self.w_door/2. + self.moving_to_static_part_distance + self.d_door),
        #                 -(self.h_door/2. + self.moving_to_static_part_distance + self.d_door)))
        # geometries.append(left_cf)

        topleft_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # topleft_mesh.transform()
        topleft_mesh.transform(self.T_O_S @ self.T_F_O)
        geometries.append(topleft_mesh)

        # axis_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # # axis_mesh.transform()
        # axis_mesh.transform(self.T_O_S@ self.T_A_O)
        # geometries.append(axis_mesh)

        point_panel_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # point_panel_mesh.transform(self.T_O_S)
        point_panel_mesh.transform(self.T_O_S @ self.T_A_O @ self.T_D_A)
        geometries.append(point_panel_mesh)

        if T_G_S is not None: # Gripper pose is w.r.t. world
            gripper_pose_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
            gripper_pose_mesh.transform(T_G_S)
            geometries.append(gripper_pose_mesh)


        A_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        A_frame.transform(self.T_A_S)
        geometries.append(A_frame)

        # o3d.visualization.draw_geometries([origin_frame, self.mesh, axis_mesh, point_panel_mesh, topleft_mesh])
        o3d.visualization.draw_geometries(geometries)

        pass


    def generate_opening_poses(self, T_G0_D0: np.ndarray, max_angle_deg: float=70., num_poses: int=10) -> list: 
        
        current_angle_deg = self.current_angle_state_deg

        opening_angles = np.linspace(current_angle_deg, max_angle_deg, num=num_poses)

        T_G0_Ss = []

        for angle_deg in opening_angles:
            self.change_door_angle(angle_deg)
            T_G0_S_current = self.get_feasible_pose_wrt_world(T_G0_D0)
            T_G0_Ss.append(T_G0_S_current)

        self.change_door_angle(current_angle_deg)
        return T_G0_Ss


    @staticmethod
    def __matrix_to_pose(T: np.ndarray):
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

if __name__ == '__main__':
    door_params = np.array([0.396, 0.496, 0.018, 0.4])

    T_A_S = np.eye(4)
    T_A_S[:3, 3] = np.array([-0.2, -0.6, door_params[1]*0.5+0.009])

    cabinet_model = Cabinet(door_params, axis_pos=-1, T_A_S=T_A_S, save_path=None)
    cabinet_model.change_door_angle(0)
    cabinet_model.update_mesh()
    # cabinet_model.get_gripper_pose_from_feasible_pose(np.eye(4))

    cabinet_model.visualize(None)




    print('Done')