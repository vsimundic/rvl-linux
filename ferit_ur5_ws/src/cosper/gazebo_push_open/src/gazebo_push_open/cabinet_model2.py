#!/usr/bin/python

import rospy
from xml.dom import minidom
import xml.etree.ElementTree as gfg
import os
from math import pi
import numpy as np
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel, GetModelState, GetJointProperties, GetJointPropertiesRequest
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, euler_from_matrix
import open3d as o3d
from copy import deepcopy
from core.transforms import rot_z, rot_y

OLD = True

class Cabinet2():  # THIS CABINET IS MADE FOR ACCESS25 PAPER
    def __init__(self,
                 # size, np.array([sx, sy, sz, static_d])
                 s: np.array = np.array([0.018, 0.4, 0.5, 0.4]),
                 r: np.array = np.array([0, 0]),  # np.array([rx, ry])
                 axis_pos: float = -1.0,
                 T_A_W: np.ndarray = np.eye(4),
                 save_path: str = None,
                 has_handle: bool = False):

        # W = World (scene)
        # O = centroid of cabinet
        # F = top left corner of static part of cabinet
        # A = Door axis
        # B = Door panel
        # D = inner left upper point on door panel

        self.sx = s[0]
        self.sy = s[1]
        self.sz = s[2]
        self.static_d = s[3]
        self.moving_to_static_part_distance = 0.005
        self.axis_distance = 0.0
        self.side = 0.018

        self.rx = r[0]
        self.ry = r[1]

        # Handle def - cylinders
        self.has_handle = has_handle
        self.handle_radius = 0.005
        self.handle_length = 0.08
        self.handle_height = 0.02
        self.handle_base_radius = 0.003
        self.handle_offset = 0.05

        self.base_link_name = 'base_cabinet_link'
        self.door_panel_link_name = 'door_link'
        self.door_joint_name = 'door_joint'
        self.cabinet_name = 'my_cabinet'
        self.handle_link_name = 'handle_link'
        self.handle_joint_name = 'handle_joint'
        self.save_path = save_path
        self.mesh_save_path = None

        self.T_A_W = T_A_W.copy()
        self.theta_deg = 0.

        self.axis_pos = axis_pos / abs(axis_pos) if axis_pos != 0 else -1.

        self.setup_matrices()

        self.xml_model = None
        self.root_urdf = None
        if self.save_path is not None:
            self.xml_model = self.generate_cabinet_urdf_from_door_panel()

        self.mesh = self.create_mesh()

    def __set_rot_pose(self):
        self.T_Arot_A = np.eye(4)
        self.T_Arot_A[:3, :3] = rot_z(np.radians(self.theta_deg))

    def __set_panel_pose(self):
        self.T_B_A = self.T_Arot_A @ self.T_B_Arot
        self.T_B_O = self.T_A_O @ self.T_B_A

        self.T_D_A = self.T_Arot_A @ self.T_D_Arot

        self.T_Arot_O = self.T_A_O @ self.T_Arot_A


    def setup_matrices(self):
        self.T_B_Arot = np.eye(4)
        self.T_B_Arot[:3, 3] = np.array([self.rx, self.ry, 0.])

        self.__set_rot_pose()  # T_Arot_A

        self.T_A_O = np.eye(4)
        self.T_A_O[:3, 3] = np.array([self.axis_pos*0.5*self.static_d, 0.5*self.sy, 0.])

        # Panel point to axis
        T_D_B = np.eye(4)
        T_D_B[:3, :3] = np.array([[0, 0, -self.axis_pos],
                                      [self.axis_pos, 0, 0],
                                      [0, -1, 0]])
        T_D_B[:3, 3] = np.array([self.sx*0.5,
                                      -self.sy*0.5,
                                      self.sz*0.5])
        
        self.T_D_Arot = self.T_B_Arot @ T_D_B


        self.__set_panel_pose()  # T_B_A, T_B_O

        # Cabinet centroid in world
        self.T_O_W = self.T_A_W @ np.linalg.inv(self.T_A_O)


        # Handle in axis frame
        self.T_H_A = np.eye(4)  # Handle in axis frame
        self.T_H_A[:3,  3] = np.array([self.axis_pos*(self.handle_height+self.sx/2.),
                                       -1*(self.sy - self.handle_offset),
                                       0.0])

    def change_door_angle(self, angle_deg):
        self.theta_deg = angle_deg
        self.__set_rot_pose()  # T_Arot_A
        self.__set_panel_pose()  # T_B_A, T_B_O


    def create_mesh(self, has_drawer: bool = True):

        # Static mesh
        x_ = self.static_d
        y_ = self.sy
        z_ = self.sx
        self.static_top_mesh = o3d.geometry.TriangleMesh.create_box(width=x_, height=y_, depth=z_)
        self.static_top_mesh.translate(-self.static_top_mesh.get_center())
        self.static_bottom_mesh = deepcopy(self.static_top_mesh)

        x_ = self.static_d
        y_ = self.sx
        z_ = self.sz - 2 * self.sx
        self.static_left_mesh = o3d.geometry.TriangleMesh.create_box(width=x_, height=y_, depth=z_)
        self.static_left_mesh.translate(-self.static_left_mesh.get_center())
        self.static_right_mesh = deepcopy(self.static_left_mesh)

        self.static_top_mesh.translate([0., 0., 0.5 * (-self.sx + self.sz)])
        self.static_bottom_mesh.translate([0., 0., -0.5 * (-self.sx + self.sz)])

        self.static_left_mesh.translate([0., 0.5 * (-self.sx + self.sy), 0.])
        self.static_right_mesh.translate([0., -0.5 * (-self.sx + self.sy), 0.])

        self.static_mesh = self.static_top_mesh + self.static_bottom_mesh + \
            self.static_left_mesh + self.static_right_mesh

        # Door panel mesh
        x_, y_, z_ = self.sx, self.sy, self.sz
        self.plate_mesh = o3d.geometry.TriangleMesh.create_box(width=x_, height=y_, depth=z_)
        self.plate_mesh.translate(-self.plate_mesh.get_center())
        self.plate_mesh.transform(self.T_B_O)

        # Drawer box mesh
        self.drawer_mesh = o3d.geometry.TriangleMesh()
        if has_drawer:
            offset = 0.03

            x_ = self.static_d + self.sx
            y_ = self.sy
            z_ = 0.305
            self.drawer_mesh = o3d.geometry.TriangleMesh.create_box(width=x_, height=y_, depth=z_)
            self.drawer_mesh.translate(-self.drawer_mesh.get_center())
            tz_ = 0.5 * self.sz + offset + 0.5 * z_
            self.drawer_mesh.translate(np.array([-self.sx * 0.5, 0.0, tz_]))

            x_ = self.static_d
            y_ = self.sy
            z_ = offset
            tz_ = 0.5 * self.sz + 0.5 * z_
            drawer_static = o3d.geometry.TriangleMesh.create_box(width=x_, height=y_, depth=z_)
            drawer_static.translate(-drawer_static.get_center())
            drawer_static.translate(np.array([0.0, 0.0, tz_]))

            self.drawer_mesh += drawer_static
        
        self.static_mesh += self.drawer_mesh

        # Handle mesh -- not debugged
        self.handle_mesh = o3d.geometry.TriangleMesh()
        if self.has_handle:
            self.handle_main_cyl_mesh = o3d.geometry.TriangleMesh.create_cylinder(
                radius=self.handle_radius, height=self.handle_length)

            # base cylinder 0
            self.handle_base_cyl0_mesh = o3d.geometry.TriangleMesh.create_cylinder(
                radius=self.handle_base_radius, height=self.handle_height)
            T_bc0_h = np.eye(4)
            T_bc0_h[:3, :3] = rot_y(np.radians(90.))
            T_bc0_h[:3, 3] = np.array(
                [-self.handle_height/2., 0.0, self.handle_length/4.])
            self.handle_base_cyl0_mesh.transform(T_bc0_h)

            # base cylinder 1
            self.handle_base_cyl1_mesh = o3d.geometry.TriangleMesh.create_cylinder(
                radius=self.handle_base_radius, height=self.handle_height)
            T_bc1_h = np.eye(4)
            T_bc1_h[:3, :3] = rot_y(np.radians(90.))
            T_bc1_h[:3, 3] = np.array(
                [-self.handle_height/2., 0.0, -self.handle_length/4.])
            self.handle_base_cyl1_mesh.transform(T_bc1_h)

            Tz_ = np.eye(4)
            Tz_[:3, :3] = rot_z(np.pi/2 - self.axis_pos*np.pi/2)
            T_H_A_ = self.T_H_A @ Tz_

            self.handle_mesh = self.handle_base_cyl0_mesh + self.handle_base_cyl1_mesh + self.handle_main_cyl_mesh
            # self.handle_mesh.translate(np.array([self.handle_height+self.sx/2., -self.axis_pos*self.sy + self.axis_pos*self.handle_offset, 0.0]))
            T_H_O = self.T_A_O @ T_H_A_
            self.handle_mesh.transform(T_H_O)
            self.handle_mesh.compute_vertex_normals()

        self.mesh = self.static_mesh + self.plate_mesh + self.handle_mesh
        self.mesh.compute_vertex_normals()

        # # debug vis
        # origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # A_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # A_rf.transform(self.T_A_O)

        # D_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        # T_D_O = self.T_A_O @ self.T_D_A
        # D_rf.transform(T_D_O)

        # o3d.visualization.draw_geometries([self.mesh, origin_rf, A_rf, D_rf])

        return self.mesh

    def save_mesh(self, filename=None, mesh=None, pose=None):
        if filename is None:
            if self.mesh_save_path is None:
                raise ValueError(
                    "Filename must be provided or mesh_save_path must be set.")
            filename = self.mesh_save_path
            if not os.path.exists(os.path.dirname(filename)):
                os.makedirs(os.path.dirname(filename))

        if mesh is None:
            self.create_mesh()
            mesh_ = deepcopy(self.mesh)
        else:
            mesh_ = mesh

        if pose is not None:
            # If a pose is provided, transform the mesh accordingly
            mesh_.transform(pose)
        else:
            mesh_.transform(np.linalg.inv(self.T_Arot_O))
        
        o3d.io.write_triangle_mesh(filename, mesh_)


    def save_mesh_without_doors(self, filename, mesh=None):
        # T_O_W = self.T_A_W @ np.linalg.inv(self.T_A_O_init)
        if mesh is None:
            static_mesh = deepcopy(self.static_mesh)
        else:
            static_mesh = mesh
        static_mesh.transform(np.linalg.inv(self.T_Arot_O))
        o3d.io.write_triangle_mesh(filename, static_mesh)


    def save_door_panel_mesh(self, filename):
        # door_panel_mesh = deepcopy(self.dd_plate_mesh)
        # door_panel_mesh.transform(np.linalg.inv(self.T_A_O @ self.T_D_A))
        door_panel_mesh = o3d.geometry.TriangleMesh.create_box(width=self.sy,
                                                               height=self.sz,
                                                               depth=self.sx)
        # door_panel_mesh.translate([-self.sy, self.sz*0.5, self.sx*0.5])
        door_panel_mesh.translate(
            [-self.sy if self.axis_pos == -1 else 0., -self.sz*0.5, -self.sx*0.5])
        T_ = np.eye(4)
        T_[:3, :3] = np.array([[0, 0, 1],
                              [1, 0, 0],
                              [0, 1, 0]])
        door_panel_mesh.transform(T_)
        # origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

        # o3d.visualization.draw_geometries([door_panel_mesh, origin_frame])

        o3d.io.write_triangle_mesh(filename, door_panel_mesh)


    def generate_cabinet_urdf_from_door_panel(self): # From old version - probably does not work right now
        static_d = self.static_d
        moving_to_static_part_distance = self.moving_to_static_part_distance
        axis_distance = self.axis_distance
        side = self.side
        bottom_panel_height = side
        w = self.sy + 2*(moving_to_static_part_distance +
                         side) + moving_to_static_part_distance
        h = self.sz + 2*moving_to_static_part_distance + side + bottom_panel_height
        d = self.sx
        base_link_name = self.base_link_name
        door_panel_link_name = self.door_panel_link_name
        axis_pos = self.axis_pos

        save_path = self.save_path

        # final visual is a union of panels (no need to make some panels shorter)
        panel_dims = np.array([[static_d, self.sy+3*moving_to_static_part_distance+2*side, bottom_panel_height],  # panel bottom
                               [static_d, d, h],  # side
                               [static_d, d, h],  # side
                               [static_d, w, d],  # top
                               [d, w, h]])  # back

        # with respect to the center of the cuboid
        panel_positions = np.array([[0., -moving_to_static_part_distance*0.5, -self.sz*0.5 - moving_to_static_part_distance - bottom_panel_height*0.5],  # bottom panel
                                    # [0., w/2. - d/2., 0], # side panel
                                    # side panel
                                    [0., self.sy*0.5 +
                                        moving_to_static_part_distance + side*0.5, 0],
                                    # [0., -(w/2. - d/2.) - moving_to_static_part_distance, 0], # side panel
                                    # side panel
                                    [0., -self.sy*0.5 - 2 *
                                        moving_to_static_part_distance - side*0.5, 0],
                                    [0., -moving_to_static_part_distance*0.5, self.sz *
                                     0.5+moving_to_static_part_distance+d*0.5],  # top
                                    [-(static_d/2.-d/2.), -moving_to_static_part_distance*0.5, 0]])  # back panel

        self.root_urdf = gfg.Element('robot')
        self.root_urdf.set('name', self.cabinet_name)

        world_link = gfg.SubElement(self.root_urdf, 'link')
        world_link.set('name', 'world')

        # Virtual joint
        virtual_joint = gfg.SubElement(self.root_urdf, 'joint')
        virtual_joint.set('name', 'virtual_joint')
        virtual_joint.set('type', 'fixed')
        vj_parent = gfg.SubElement(virtual_joint, 'parent')
        vj_parent.set('link', 'world')
        vj_child = gfg.SubElement(virtual_joint, 'child')
        vj_child.set('link', base_link_name)
        vj_origin = gfg.SubElement(virtual_joint, 'origin')
        vj_origin.set('xyz', '0.0 0.0 0.0')
        vj_origin.set('rpy', '0.0 0.0 0.0')

        base_door_link = gfg.SubElement(self.root_urdf, 'link')
        base_door_link.set('name', base_link_name)

        # Cabinet panels
        for i in range(panel_positions.shape[0]):
            panel_visual = gfg.SubElement(base_door_link, 'visual')

            pl_visual_origin = gfg.SubElement(panel_visual, 'origin')
            pl_visual_origin.set('xyz', '{} {} {}'.format(
                panel_positions[i, 0], panel_positions[i, 1], panel_positions[i, 2]))
            pl_visual_origin.set('rpy', '0.0 0.0 0.0')

            pl_visual_geom = gfg.SubElement(panel_visual, 'geometry')
            pl_visual_geom_box = gfg.SubElement(pl_visual_geom, 'box')
            pl_visual_geom_box.set('size', '{} {} {}'.format(
                panel_dims[i, 0], panel_dims[i, 1], panel_dims[i, 2]))

            panel_collision = gfg.SubElement(base_door_link, 'collision')
            panel_collision.set('name', base_link_name+'_collision_%d' % i)
            pl_collision_origin = gfg.SubElement(panel_collision, 'origin')
            pl_collision_origin.set('xyz', '{} {} {}'.format(
                panel_positions[i, 0], panel_positions[i, 1], panel_positions[i, 2]))
            pl_collision_origin.set('rpy', '0.0 0.0 0.0')

            pl_collision_geom = gfg.SubElement(panel_collision, 'geometry')
            pl_collision_geom_box = gfg.SubElement(pl_collision_geom, 'box')
            pl_collision_geom_box.set('size', '{} {} {}'.format(
                panel_dims[i, 0], panel_dims[i, 1], panel_dims[i, 2]))

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

        # Door panel
        # Dimensions
        # door_panel_dims = [d, w - 2*(moving_to_static_part_distance + side), h - 2 * (moving_to_static_part_distance + side)]
        door_panel_dims = [self.sx, self.sy, self.sz]
        door_panel_position = [static_d/2. - d/2., 0., 0.]

        door_panel_link = gfg.SubElement(self.root_urdf, 'link')
        door_panel_link.set('name', 'door_link')

        door_panel_visual = gfg.SubElement(door_panel_link, 'visual')
        dpl_visual_origin = gfg.SubElement(door_panel_visual, 'origin')

        # Old - working
        if OLD:
            dpl_visual_origin.set(
                'xyz', '0.0 {} 0.0'.format(-1*(door_panel_dims[1]/2. - axis_distance)))
        else:
            # New - in coll when fully open
            if self.axis_pos == 1:
                dpl_visual_origin.set(
                    'xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))
            else:
                dpl_visual_origin.set(
                    'xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))

        # dpl_visual_origin.set('xyz', '0.0 {} 0.0'.format(0))
        dpl_visual_geom = gfg.SubElement(door_panel_visual, 'geometry')
        dpl_visual_geom_box = gfg.SubElement(dpl_visual_geom, 'box')
        dpl_visual_geom_box.set('size', '{} {} {}'.format(
            door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))

        door_panel_collision = gfg.SubElement(door_panel_link, 'collision')
        door_panel_collision.set(
            'name', self.door_panel_link_name+'_collision')
        dpl_collision_origin = gfg.SubElement(door_panel_collision, 'origin')

        # Old - working
        if OLD:
            dpl_collision_origin.set(
                'xyz', '0.0 {} 0.0'.format(-1*(door_panel_dims[1]/2. - axis_distance)))
        else:
            # New - in coll when fully open
            if self.axis_pos == 1:
                dpl_collision_origin.set(
                    'xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))
            else:
                dpl_collision_origin.set(
                    'xyz', '0.0 {} 0.0'.format(-door_panel_dims[1]/2.))

        # dpl_collision_origin.set('xyz', '0.0 {} 0.0'.format(axis_pos*(door_panel_dims[1]/2. - axis_distance)))
        dpl_collision_geom = gfg.SubElement(door_panel_collision, 'geometry')
        dpl_collision_geom_box = gfg.SubElement(dpl_collision_geom, 'box')
        dpl_collision_geom_box.set('size', '{} {} {}'.format(
            door_panel_dims[0], door_panel_dims[1], door_panel_dims[2]))

        # dpl_collision_contact_coefs = gfg.SubElement(door_panel_collision, 'contact_coefficients')
        # dpl_collision_contact_coefs.set('mu', '2.0')
        # dpl_collision_contact_coefs.set('kp', '100000')
        # dpl_collision_contact_coefs.set('kd', '10')

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

        # Door joint
        door_joint = gfg.SubElement(self.root_urdf, 'joint')
        door_joint.set('name', self.door_joint_name)
        door_joint.set('type', 'revolute')

        j0_origin = gfg.SubElement(door_joint, 'origin')
        # j0_origin.set('xyz', '{} {} 0'.format(static_d/2 - d/2, axis_pos*(door_panel_dims[1]/2. - axis_distance)))
        j0_origin.set('xyz', '{} {} {}'.format(
            self.T_A_O[0, 3], self.T_A_O[1, 3],  self.T_A_O[2, 3]))
        j0_origin.set('rpy', '%f %f %f' % euler_from_matrix(self.T_A_O))
        # j0_origin.set('xyz', '{} {} 0'.format(static_d/2 - d/2, -(-door_panel_dims[1]/2. + axis_distance)))
        # j0_origin.set('rpy', '{} {} {}'.format(0,0, np.radians(180)))
        # j0_origin.set('rpy', '{} {} {}'.format(0, 0, 0))
        j0_axis = gfg.SubElement(door_joint, 'axis')
        j0_axis.set('xyz', '0 0 1')
        j0_parent = gfg.SubElement(door_joint, 'parent')
        j0_parent.set('link', base_link_name)
        j0_child = gfg.SubElement(door_joint, 'child')
        j0_child.set('link', door_panel_link_name)
        j0_limit = gfg.SubElement(door_joint, 'limit')
        j0_limit.set('effort', '50')

        if axis_pos == 1:
            j0_limit.set('lower', '0')
            j0_limit.set('upper', '{}'.format(pi/2. + np.deg2rad(5.)))
        else:
            j0_limit.set('lower', '{}'.format(-pi/2. - np.deg2rad(5.)))
            j0_limit.set('upper', '0')

        j0_limit.set('velocity', '10')
        j0_dynamics = gfg.SubElement(door_joint, 'dynamics')
        j0_dynamics.set('friction', '3.5')
        j0_dynamics.set('damping', '3.5')

        # gazebo_ = gfg.SubElement(self.root_urdf, 'gazebo')
        # gazebo_.set('reference', base_link_name)
        # gazebo_material = gfg.SubElement(gazebo_, 'material')
        # gazebo_material.text = 'Gazebo/Blue'

        # Handle
        if self.has_handle:

            handle_link = gfg.SubElement(self.root_urdf, 'link')
            handle_link.set('name', self.handle_link_name)

            handle_visual = gfg.SubElement(handle_link, 'visual')
            h_visual_origin = gfg.SubElement(handle_visual, 'origin')
            h_visual_origin.set('xyz', '0.0 0.0 0.0')
            h_visual_origin.set('rpy', '0.0 0.0 0.0')

            h_visual_geom = gfg.SubElement(handle_visual, 'geometry')
            h_visual_geom_cyl = gfg.SubElement(h_visual_geom, 'cylinder')
            h_visual_geom_cyl.set('length', '{}'.format(self.handle_length))
            h_visual_geom_cyl.set('radius', '{}'.format(self.handle_radius))

            handle_collision = gfg.SubElement(handle_link, 'collision')
            handle_collision.set('name', self.handle_link_name+'_collision_0')
            h_collision_origin = gfg.SubElement(handle_collision, 'origin')
            h_collision_origin.set('xyz', '0.0 0.0 0.0')
            h_collision_origin.set('rpy', '0.0 0.0 0.0')

            h_collision_geom = gfg.SubElement(handle_collision, 'geometry')
            h_collision_geom_cyl = gfg.SubElement(h_collision_geom, 'cylinder')
            h_collision_geom_cyl.set('length', '{}'.format(self.handle_length))
            h_collision_geom_cyl.set('radius', '{}'.format(self.handle_radius))

            # base cylinder 0
            handle_base0_visual = gfg.SubElement(handle_link, 'visual')
            hb0_visual_origin = gfg.SubElement(handle_base0_visual, 'origin')
            hb0_visual_origin.set(
                'xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, self.handle_length/4.))
            hb0_visual_origin.set('rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))

            hb0_visual_geom = gfg.SubElement(handle_base0_visual, 'geometry')
            hb0_visual_geom_cyl = gfg.SubElement(hb0_visual_geom, 'cylinder')
            hb0_visual_geom_cyl.set('length', '{}'.format(self.handle_height))
            hb0_visual_geom_cyl.set(
                'radius', '{}'.format(self.handle_base_radius))

            handle_base0_collision = gfg.SubElement(handle_link, 'collision')
            handle_base0_collision.set(
                'name', self.handle_link_name+'_collision_1')
            hb0_collision_origin = gfg.SubElement(
                handle_base0_collision, 'origin')
            hb0_collision_origin.set(
                'xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, self.handle_length/4.))
            hb0_collision_origin.set(
                'rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))

            hb0_collision_geom = gfg.SubElement(
                handle_base0_collision, 'geometry')
            hb0_collision_geom_cyl = gfg.SubElement(
                hb0_collision_geom, 'cylinder')
            hb0_collision_geom_cyl.set(
                'length', '{}'.format(self.handle_height/2.))
            hb0_collision_geom_cyl.set(
                'radius', '{}'.format(self.handle_base_radius))

            # base cylinder 1
            handle_base1_visual = gfg.SubElement(handle_link, 'visual')
            hb1_visual_origin = gfg.SubElement(handle_base1_visual, 'origin')
            hb1_visual_origin.set(
                'xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, -self.handle_length/4.))
            hb1_visual_origin.set('rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))

            hb1_visual_geom = gfg.SubElement(handle_base1_visual, 'geometry')
            hb1_visual_geom_cyl = gfg.SubElement(hb1_visual_geom, 'cylinder')
            hb1_visual_geom_cyl.set('length', '{}'.format(self.handle_height))
            hb1_visual_geom_cyl.set(
                'radius', '{}'.format(self.handle_base_radius))

            handle_base1_collision = gfg.SubElement(handle_link, 'collision')
            handle_base1_collision.set(
                'name', self.handle_link_name+'_collision_2')
            hb1_collision_origin = gfg.SubElement(
                handle_base1_collision, 'origin')
            hb1_collision_origin.set(
                'xyz', '{} {} {}'.format(-self.handle_height/2., 0.0, -self.handle_length/4.))
            hb1_collision_origin.set(
                'rpy', '{} {} {}'.format(0.0, np.pi/2., 0.0))

            hb1_collision_geom = gfg.SubElement(
                handle_base1_collision, 'geometry')
            hb1_collision_geom_cyl = gfg.SubElement(
                hb1_collision_geom, 'cylinder')
            hb1_collision_geom_cyl.set(
                'length', '{}'.format(self.handle_height))
            hb1_collision_geom_cyl.set(
                'radius', '{}'.format(self.handle_base_radius))

            # Moment of inertia for the handle (as a box)
            h_inertial = gfg.SubElement(handle_link, 'inertial')
            m_h = 0.05
            ixx_h = 1/12.*m_h*((2*self.handle_radius) **
                               2 + (self.handle_length)**2)
            iyy_h = 1/12.*m_h * \
                ((self.handle_radius+self.handle_height)
                 ** 2 + (self.handle_length)**2)
            izz_h = 1/12.*m_h*((2*self.handle_radius)**2 +
                               (self.handle_radius+self.handle_height)**2)

            h_inertial_mass = gfg.SubElement(h_inertial, 'mass')
            h_inertial_mass.set('value', '%f' % m_h)
            h_inertial_inertia = gfg.SubElement(h_inertial, 'inertia')
            h_inertial_inertia.set('ixx', '%f' % ixx_h)
            h_inertial_inertia.set('ixy', '0.0')
            h_inertial_inertia.set('ixz', '0.0')
            h_inertial_inertia.set('iyy', '%f' % iyy_h)
            h_inertial_inertia.set('iyz', '0.0')
            h_inertial_inertia.set('izz', '%f' % izz_h)

            # Handle joint
            handle_joint = gfg.SubElement(self.root_urdf, 'joint')
            handle_joint.set('name', self.handle_joint_name)
            handle_joint.set('type', 'fixed')
            h_j_origin = gfg.SubElement(handle_joint, 'origin')
            h_j_origin.set('xyz', '{} {} {}'.format(
                self.T_H_A[0, 3], self.T_H_A[1, 3], self.T_H_A[2, 3]))

            Tz_ = np.eye(4)
            Tz_[:3, :3] = rot_z(np.pi/2 - axis_pos*np.pi/2)

            h_j_origin.set('rpy', '%f %f %f' % euler_from_matrix(Tz_))
            h_j_parent = gfg.SubElement(handle_joint, 'parent')
            h_j_parent.set('link', door_panel_link_name)
            h_j_child = gfg.SubElement(handle_joint, 'child')
            h_j_child.set('link', self.handle_link_name)

        # Contact sensor
        gazebo_sensor_ = gfg.SubElement(self.root_urdf, 'gazebo')
        gazebo_sensor_.set('reference', base_link_name)
        gazebo_sensor_sensor = gfg.SubElement(gazebo_sensor_, 'sensor')
        gazebo_sensor_sensor.set('name', 'main_sensor')
        gazebo_sensor_sensor.set('type', 'contact')
        gazebo_sensor_sensor_selfCollide = gfg.SubElement(
            gazebo_sensor_sensor, 'selfCollide')
        gazebo_sensor_sensor_selfCollide.text = 'true'
        gazebo_sensor_sensor_alwaysOn = gfg.SubElement(
            gazebo_sensor_sensor, 'alwaysOn')
        gazebo_sensor_sensor_alwaysOn.text = 'true'
        gazebo_sensor_sensor_updateRate = gfg.SubElement(
            gazebo_sensor_sensor, 'updateRate')
        gazebo_sensor_sensor_updateRate.text = '5.0'
        gazebo_sensor_sensor_material = gfg.SubElement(
            gazebo_sensor_sensor, 'material')
        gazebo_sensor_sensor_material.text = 'Gazebo/Red'
        gazebo_sensor_sensor_contact = gfg.SubElement(
            gazebo_sensor_sensor, 'contact')

        gazebo_sensor_sensor_contact_0 = gfg.SubElement(
            gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_0.text = base_link_name + \
            '_collision_%d' % 0 + '_collision'
        gazebo_sensor_sensor_contact_1 = gfg.SubElement(
            gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_1.text = base_link_name + \
            '_collision_%d' % 1 + '_collision'
        gazebo_sensor_sensor_contact_2 = gfg.SubElement(
            gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_2.text = base_link_name + \
            '_collision_%d' % 2 + '_collision'
        gazebo_sensor_sensor_contact_3 = gfg.SubElement(
            gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_3.text = base_link_name + \
            '_collision_%d' % 3 + '_collision'
        gazebo_sensor_sensor_contact_4 = gfg.SubElement(
            gazebo_sensor_sensor_contact, 'collision')
        gazebo_sensor_sensor_contact_4.text = base_link_name + \
            '_collision_%d' % 4 + '_collision'

        gazebo_sensor_sensor_plugin = gfg.SubElement(
            gazebo_sensor_sensor, 'plugin')
        gazebo_sensor_sensor_plugin.set('name', 'gazebo_ros_bumper_controller')
        gazebo_sensor_sensor_plugin.set('filename', 'libgazebo_ros_bumper.so')
        gazebo_sensor_sensor_plugin_bumperTopicName = gfg.SubElement(
            gazebo_sensor_sensor_plugin, 'bumperTopicName')
        gazebo_sensor_sensor_plugin_bumperTopicName.text = '/contact'
        gazebo_sensor_sensor_plugin_frameName = gfg.SubElement(
            gazebo_sensor_sensor_plugin, 'frameName')
        gazebo_sensor_sensor_plugin_frameName.text = 'world'

        # Door contact sensor
        gazebo_sensor_door = gfg.SubElement(self.root_urdf, 'gazebo')
        gazebo_sensor_door.set('reference', door_panel_link_name)
        gazebo_sensor_doorsensor = gfg.SubElement(gazebo_sensor_door, 'sensor')
        gazebo_sensor_doorsensor.set('name', 'main_sensor')
        gazebo_sensor_doorsensor.set('type', 'contact')
        gazebo_sensor_doorsensor_selfCollide = gfg.SubElement(
            gazebo_sensor_doorsensor, 'selfCollide')
        gazebo_sensor_doorsensor_selfCollide.text = 'true'
        gazebo_sensor_doorsensor_alwaysOn = gfg.SubElement(
            gazebo_sensor_doorsensor, 'alwaysOn')
        gazebo_sensor_doorsensor_alwaysOn.text = 'true'
        gazebo_sensor_doorsensor_updateRate = gfg.SubElement(
            gazebo_sensor_doorsensor, 'updateRate')
        gazebo_sensor_doorsensor_updateRate.text = '5.0'
        gazebo_sensor_doorsensor_material = gfg.SubElement(
            gazebo_sensor_doorsensor, 'material')
        gazebo_sensor_doorsensor_material.text = 'Gazebo/Red'
        gazebo_sensor_doorsensor_contact = gfg.SubElement(
            gazebo_sensor_doorsensor, 'contact')

        gazebo_sensor_doorsensor_contact_0 = gfg.SubElement(
            gazebo_sensor_doorsensor_contact, 'collision')
        gazebo_sensor_doorsensor_contact_0.text = self.door_panel_link_name + \
            '_collision' + '_collision'

        gazebo_sensor_doorsensor_plugin = gfg.SubElement(
            gazebo_sensor_doorsensor, 'plugin')
        gazebo_sensor_doorsensor_plugin.set(
            'name', 'gazebo_ros_bumper_controller')
        gazebo_sensor_doorsensor_plugin.set(
            'filename', 'libgazebo_ros_bumper.so')
        gazebo_sensor_doorsensor_plugin_bumperTopicName = gfg.SubElement(
            gazebo_sensor_doorsensor_plugin, 'bumperTopicName')
        gazebo_sensor_doorsensor_plugin_bumperTopicName.text = '/contact_door'
        gazebo_sensor_doorsensor_plugin_frameName = gfg.SubElement(
            gazebo_sensor_doorsensor_plugin, 'frameName')
        gazebo_sensor_doorsensor_plugin_frameName.text = 'contact_door'

        # Add contact params to the door
        gazebo_contact_door = gfg.SubElement(self.root_urdf, 'gazebo')
        gazebo_contact_door.set('reference', door_panel_link_name)
        gazebo_contact_door_mu = gfg.SubElement(gazebo_contact_door, 'mu')
        gazebo_contact_door_mu.text = '3.0'
        gazebo_contact_door_mu2 = gfg.SubElement(gazebo_contact_door, 'mu2')
        gazebo_contact_door_mu2.text = '3.0'
        gazebo_contact_door_kp = gfg.SubElement(gazebo_contact_door, 'kp')
        gazebo_contact_door_kp.text = '10000000'
        gazebo_contact_door_kd = gfg.SubElement(gazebo_contact_door, 'kd')
        gazebo_contact_door_kd.text = '100'
        # gazebo_contact_door_slip1 = gfg.SubElement(gazebo_contact_door, 'slip1')
        # gazebo_contact_door_slip1.text = '0.5'
        # gazebo_contact_door_slip2 = gfg.SubElement(gazebo_contact_door, 'slip2')
        # gazebo_contact_door_slip2.text = '0.5'

        # gazebo_contact_door_min_depth = gfg.SubElement(gazebo_contact_door, 'min_depth')
        # gazebo_contact_door_min_depth.text = '0.001'

        # Prettify for writing in a file
        xmlstr = minidom.parseString(gfg.tostring(
            self.root_urdf)).toprettyxml(indent='\t')

        if save_path:
            if not os.path.exists(os.path.dirname(save_path)):
                os.makedirs(os.path.dirname(save_path))
            with open(save_path, 'w') as f:
                f.write(xmlstr)

        return xmlstr


if __name__ == '__main__':
    s = np.array([0.018, 0.395, 0.495, 0.4])

    Tz = np.eye(4)
    T_A_W = np.eye(4)
    T_A_W = Tz
    # T_A_W[:3,:3] = np.array([[-1,0,0],
    #                         [0,-1,0],
    #                         [0,0,1]])
    T_A_W[:3, 3] = np.array([-0.3, -0.4, 0.278])

    cabinet_model = Cabinet2(s,
                            axis_pos=-1,
                            r=np.array([-0.009, -0.5*s[1]]),
                            T_A_W=T_A_W,
                            save_path=None,
                            has_handle=False)
    cabinet_model.create_mesh()