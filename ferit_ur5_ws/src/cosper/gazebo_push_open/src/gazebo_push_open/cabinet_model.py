#!/usr/bin/python

import rospy
from xml.dom import minidom
import xml.etree.ElementTree as gfg
import os
from math import pi
import numpy as np
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix
import open3d as o3d

class Cabinet():
    def __init__(self, w_door: float, h_door: float, d_door: float=0.018, static_d: float=0.3, axis_pos: int=1, save_path:str=None):
        self.w_door = w_door
        self.h_door = h_door
        self.d_door = d_door
        self.static_d = static_d
        self.moving_to_static_part_distance = 0.005
        self.axis_distance = 0.01
        self.static_side_width = 0.018
        self.base_link_name = 'base_cabinet_link'
        self.door_panel_link_name = 'door_link'
        self.door_joint_name = 'door_joint'
        self.cabinet_name = 'my_cabinet'
        self.save_path = save_path

        if not axis_pos == 1 and not axis_pos == -1:
            self.axis_pos = 1
        else:
            self.axis_pos = axis_pos

        self.xml_model = self.generate_cabinet_urdf_from_door_panel()


    def generate_cabinet_urdf_from_door_panel(self):
        static_d = self.static_d
        moving_to_static_part_distance = self.moving_to_static_part_distance
        axis_distance = self.axis_distance
        static_side_width = self.static_side_width
        w = self.w_door + 2*(moving_to_static_part_distance + static_side_width)
        h = self.h_door + 2*(moving_to_static_part_distance + static_side_width)
        d = self.d_door
        base_link_name = self.base_link_name
        door_panel_link_name = self.door_panel_link_name
        axis_pos = self.axis_pos

        save_path = self.save_path

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
        dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(-axis_pos*(door_panel_dims[1]/2. - axis_distance)))
        dpl_visual_geom = gfg.SubElement(door_panel_visual, 'geometry')
        dpl_visual_geom_box = gfg.SubElement(dpl_visual_geom, 'box')
        dpl_visual_geom_box.set('size', '{} {} {}'.format(door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))

        door_panel_collision = gfg.SubElement(door_panel_link, 'collision')
        dpl_collision_origin = gfg.SubElement(door_panel_collision, 'origin')
        dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(-axis_pos*(door_panel_dims[1]/2. - axis_distance)))
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
        j0_origin.set('xyz', '{} {} 0'.format(static_d/2 - d/2, axis_pos*(-door_panel_dims[1]/2. + axis_distance)))
        j0_origin.set('rpy', '{} {} {}'.format(0,0, np.radians(180)))
        j0_axis = gfg.SubElement(door_joint, 'axis')
        j0_axis.set('xyz', '0 0 1')
        j0_parent = gfg.SubElement(door_joint, 'parent')
        j0_parent.set('link', base_link_name)
        j0_child = gfg.SubElement(door_joint, 'child')
        j0_child.set('link', door_panel_link_name)
        j0_limit = gfg.SubElement(door_joint, 'limit')
        j0_limit.set('effort', '50')

        if axis_pos == 1:
            j0_limit.set('lower', '{}'.format(-pi/2.))
            j0_limit.set('upper', '0')
        elif axis_pos == -1:
            j0_limit.set('lower', '{}'.format(0))
            j0_limit.set('upper', '{}'.format(pi/2.))

        j0_limit.set('velocity', '10')
        j0_dynamics = gfg.SubElement(door_joint, 'dynamics')
        j0_dynamics.set('friction', '1.0')

        gazebo_ = gfg.SubElement(root, 'gazebo')
        gazebo_.set('reference', 'my_cabinet')
        gazebo_material = gfg.SubElement(gazebo_, 'material')
        gazebo_material.text = 'Gazebo/Blue'


        # Prettify for writing in a file
        xmlstr = minidom.parseString(gfg.tostring(root)).toprettyxml(indent='\t')

        if save_path:
            with open (save_path, 'w') as f:
                f.write(xmlstr)

        return xmlstr


    def get_centroid_to_axis_pose(self):
        # TXA
        TXA = np.eye(4)
        TXA[:3, 3] = np.array([self.static_d/2 - self.d_door/2, self.axis_pos*(-self.w_door/2. + self.axis_distance), 0.])
        return TXA


    def get_world_pose(self, x: float, y: float, z: float, angle_deg: float):
        TA0 = np.eye(4)
        TA0[:3, 3] = np.array([x, y, z])

        theta0_deg = 180
        Tz = np.eye(4)
        theta = np.radians(angle_deg + theta0_deg)
        s = np.sin(theta)
        c = np.cos(theta)
        Tz[:3, :3] = np.array([[c, -s, 0.],
                                [s, c, 0.],
                                [0., 0., 1.]])

        TXA = self.get_centroid_to_axis_pose()
        TX0 = TA0 @ TXA @ Tz

        return TX0


    def spawn_model_gazebo(self, x: float, y: float, z: float, angle_deg: float):
        TX0 = self.get_world_pose(x, y, z, angle_deg)

        init_pose = Pose()
        init_pose.position.x = TX0[0, 3]
        init_pose.position.y = TX0[1, 3]
        init_pose.position.z = TX0[2, 3]

        q = quaternion_from_matrix(TX0)

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


    def delete_model_gazebo(self):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            _ = spawn_urdf_model_proxy(self.cabinet_name)
            rospy.loginfo('URDF deleted successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to delete model: %s' % e)


    def move_model_joint_gazebo(self, joint_value_deg):

        if self.axis_pos == 1:
            if joint_value_deg < -90.:
                joint_value_deg = -90.
            elif joint_value_deg > 0.:
                joint_value_deg = -0.
        elif self.axis_pos == -1:
            if joint_value_deg < 0.:
                joint_value_deg = 0.
            elif joint_value_deg > 90.:
                joint_value_deg = 90.
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


    # def get_gripper_pose_from_feasible_pose(self, feasible_pose: np.ndarray, TX0: np.ndarray, TB0: np.ndarray):
    def get_gripper_pose_from_feasible_pose(self, feasible_pose: np.ndarray):
        # feasible pose = TGS

        TSX = np.eye(4)
        
        TSX[:3, :3] = np.array([[0, 0, -1],
                                [1, 0, 0],
                                [0, -1, 0]])
        TSX[:3, 3] = np.array([self.static_d/2., 
                               -(self.w_door/2. + self.moving_to_static_part_distance + self.d_door/2.), 
                                self.h_door/2. + self.moving_to_static_part_distance + self.d_door/2.])

        # X_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        # S_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # S_mesh.transform(TSX)

        # o3d.visualization.draw_geometries([X_mesh, S_mesh])

        # return TB0.T @ TX0 @ TSX @ feasible_pose
        return TSX @ feasible_pose

        


if __name__ == '__main__':

    cabinet_model = Cabinet(0.3, 0.5, 0.018, 0.3, 1, None)
    cabinet_model.get_gripper_pose_from_feasible_pose(np.eye(4))

    print('Done')