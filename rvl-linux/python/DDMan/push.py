import numpy as np
from numpy.core.multiarray import empty
from numpy.lib.shape_base import expand_dims
import open3d as o3d
import matplotlib.pyplot as plt

import sys
import os
sys.path.append('/home/RVLuser/rvl-linux/modules/RVLPY')
os.chdir('/home/RVLuser/rvl-linux/python/DDMan')
from importlib import reload


import rvlpyutil as rvl
from DDMan import vn_model as vn
# import vn_model as vn
import copy

def random_orientations(num_viewpoints = 300, num_rot_angles = 12):
    # Constants.

    num_orientations = num_viewpoints * num_rot_angles
    d_rot_angle = 2.0 * np.pi / num_rot_angles

    # Generate viewpoints by sampling the unit sphere.

    # cube_samples = np.random.rand(num_viewpoints, 3)
    rng = np.random.default_rng(12345)
    cube_samples = rng.random((num_viewpoints, 3))
    cube_samples[:,0] = 2.0 * cube_samples[:,0] - 1.0
    cube_samples[:,2] = 2.0 * cube_samples[:,2] - 1.0
    sphere_samples = cube_samples / np.linalg.norm(cube_samples, axis=1)[:,np.newaxis]

    # Sphere sampling visualization.

    # unit_sphere_pcd = o3d.geometry.PointCloud()
    # unit_sphere_pcd.points = o3d.utility.Vector3dVector(sphere_samples)
    # unit_sphere_pcd.paint_uniform_color([0.0, 1.0, 0.0])

    # o3d.visualization.draw_geometries([unit_sphere_pcd])

    # Generate random rotation matrices with z-axis unifomly distributed over the unit sphere (viewpoint rotation matrices).

    rot_mx = np.zeros((num_viewpoints, 3, 3))
    rot_mx[:,:,2] = sphere_samples
    axis_idx = np.argmin(np.abs(sphere_samples), axis=1)
    v = np.zeros((num_viewpoints, 3))
    np.put_along_axis(v, axis_idx[:,np.newaxis], 1.0, axis=1)
    rot_mx[:,:,0] = np.cross(sphere_samples,v)
    rot_mx[:,:,0] = rot_mx[:,:,0] / np.linalg.norm(rot_mx[:,:,0], axis=1)[:,np.newaxis]
    rot_mx[:,:,1] = np.cross(rot_mx[:,:,2], rot_mx[:,:,0])

    # Multiply rotation matrices num_rot_angles times.

    # rot_mx = np.tile(np.reshape(rot_mx, (num_viewpoints, 9)), num_rot_angles)
    # rot_mx = np.reshape(rot_mx, (num_orientations, 3, 3))

    # Generate rotation matrices representing random rotations about z-axis (roll rotation matrices).
    
    # first_rot_angle = d_rot_angle * np.random.rand(num_viewpoints)
    first_rot_angle = d_rot_angle * rng.random(num_viewpoints)
    Rz = np.zeros((num_viewpoints, num_rot_angles, 3, 3))
    Rz[:,:,2,2] = 1.0
    for rot_angle_idx in range(num_rot_angles):
        rot_angle = first_rot_angle + rot_angle_idx * d_rot_angle
        cs = np.cos(rot_angle)
        sn = np.sin(rot_angle)
        Rz[:,rot_angle_idx,0,0] = cs
        Rz[:,rot_angle_idx,0,1] = -sn
        Rz[:,rot_angle_idx,1,0] = sn
        Rz[:,rot_angle_idx,1,1] = cs

    # Multiply viewpoint rotation matrices with the roll rotation matrices.

    rot_mx = rot_mx[:,np.newaxis,:,:] @ Rz
    rot_mx = np.reshape(rot_mx, (num_orientations, 3, 3))
    
    return rot_mx

class push():
    def __init__(self, dd, tool):
        self.dd = dd
        self.tool = tool

    def intersection(self, tcs_vertices, tcs_normal, d_tcs, dcs_vertices, vision_tolerance):
        # Intersections of the orthogonal projections of the TCS edges onto the supporting plane of DCS with the DCS edges.

        A_dcs = np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]])
        # d_dcs = np.array([self.dd.dd_contact_surface_params[0], self.dd.dd_contact_surface_params[1], -vision_tolerance, -vision_tolerance])
        d_dcs = (A_dcs @ dcs_vertices.T).max(1)
        e_tcs = A_dcs @ tcs_vertices.T - d_dcs[:, np.newaxis]
        next_vertex_idx = np.array([1, 3, 0, 2]).astype(np.int32)
        v = tcs_vertices[next_vertex_idx, :] - tcs_vertices
        f = A_dcs @ v.T
        edge_intersection = (np.abs(f) > 1e-10)
        s = -np.ones((4, 4))
        s[edge_intersection] = -e_tcs[edge_intersection] / f[edge_intersection]
        edge_intersection = np.logical_and(edge_intersection, s >= 0.0)
        edge_intersection = np.logical_and(edge_intersection, s <= 1.0)
        edge_intersection_points = tcs_vertices[np.newaxis,:,:] + s[:,:,np.newaxis] * v[np.newaxis,:,:]
        e_edge_intersection_points = edge_intersection_points @ A_dcs.T - d_dcs
        edge_intersection_points_out = (e_edge_intersection_points >= 0.0)
        for dcs_edge_idx in range(4):
            edge_intersection_points_out[dcs_edge_idx, :, dcs_edge_idx] = False
        edge_intersection_points_out = np.any(edge_intersection_points_out, axis=2)
        edge_intersection = np.logical_and(edge_intersection, np.logical_not(edge_intersection_points_out))
        intersection_vertices = edge_intersection_points[edge_intersection,:]

        # Orthogonal projection planes.

        A_tcs = np.cross(np.array([0, 0, 1]), v)
        A_tcs /= np.expand_dims(np.linalg.norm(A_tcs, axis=1), 1)    
        d_tcs_pp = np.sum(A_tcs * tcs_vertices, axis=1)

        # DCS vertices inside the orthogonal projection of TCS.

        e_dcs = dcs_vertices @ A_tcs.T - d_tcs_pp
        dcs_vertices_inside_tcs = dcs_vertices[np.all(e_dcs <= 0, axis=1),:]
        if dcs_vertices_inside_tcs.shape[0] > 0:
            e_dcs_vertices_inside_tcs = d_tcs - dcs_vertices_inside_tcs @ tcs_normal
            e_dcs_vertices_inside_tcs = e_dcs_vertices_inside_tcs[0,:]
            s = e_dcs_vertices_inside_tcs / tcs_normal[2]
            tcs_points_projecting_to_dcs_vertices = dcs_vertices_inside_tcs.copy()
            tcs_points_projecting_to_dcs_vertices[:,2] += s
            intersection_vertices = np.concatenate((intersection_vertices, tcs_points_projecting_to_dcs_vertices), 0)

        # TCS vertices with orthogonal projection to DCS.
    
        tcs_vertices_projecting_to_dcs = tcs_vertices[np.all(e_tcs <= 0, axis=0),:]
        if tcs_vertices_projecting_to_dcs.shape[0] > 0:
            intersection_vertices = np.concatenate((intersection_vertices, tcs_vertices_projecting_to_dcs), 0)

        return intersection_vertices


    def z_shift(self, tcs_params, vision_tolerance, R, contact_point, visualization = False):
        # Tool contact surface (TCS).

        tcs_vertices_TCS = np.stack((tcs_params[0,:], tcs_params[0,:], tcs_params[1,:], tcs_params[1,:]))
        tcs_vertices_TCS[1,1] = -tcs_vertices_TCS[1,1]
        tcs_vertices_TCS[3,1] = -tcs_vertices_TCS[3,1]
        tcs_triangles = np.array([[0, 1, 2], [1, 3, 2]]).astype(np.int32)
        tcs_normal_TCS = np.cross(tcs_vertices_TCS[1,:] - tcs_vertices_TCS[0,:], tcs_vertices_TCS[2,:] - tcs_vertices_TCS[0,:])
        tcs_normal_TCS /= np.linalg.norm(tcs_normal_TCS)

        # Door/drawer contact surface (DCS).

        if self.dd.dd_opening_direction > 0.0:
            dcs_vertices = np.array([[0.0, 0.0, 0.0], [self.dd.dd_contact_surface_params[0], 0.0, 0.0],
                [self.dd.dd_contact_surface_params[0], self.dd.dd_contact_surface_params[1], 0.0], 
                [0.0, self.dd.dd_contact_surface_params[1], 0.0]])
            dcs_vertices_in = dcs_vertices.copy()
            dcs_vertices_in[0,0] = vision_tolerance
            dcs_vertices_in[0,1] = vision_tolerance
            dcs_vertices_in[1,1] = vision_tolerance
            dcs_vertices_in[3,0] = vision_tolerance
            dcs_vertices_out = dcs_vertices.copy()
            dcs_vertices_out[0,0] = -vision_tolerance
            dcs_vertices_out[0,1] = -vision_tolerance
            dcs_vertices_out[1,1] = -vision_tolerance
            dcs_vertices_out[3,0] = -vision_tolerance
        else:
            dcs_vertices = np.array([[0.0, 0.0, 0.0], [0.0, self.dd.dd_contact_surface_params[1], 0.0],
                [-self.dd.dd_contact_surface_params[0], self.dd.dd_contact_surface_params[1], 0.0], 
                [-self.dd.dd_contact_surface_params[0], 0.0, 0.0]])
            dcs_vertices_in = dcs_vertices.copy()
            dcs_vertices_in[0,0] = -vision_tolerance
            dcs_vertices_in[0,1] = vision_tolerance
            dcs_vertices_in[1,0] = -vision_tolerance
            dcs_vertices_in[3,1] = vision_tolerance
            dcs_vertices_out = dcs_vertices.copy()
            dcs_vertices_out[0,0] = vision_tolerance
            dcs_vertices_out[0,1] = -vision_tolerance
            dcs_vertices_out[1,0] = vision_tolerance
            dcs_vertices_out[3,1] = -vision_tolerance
        dcs_triangles = np.array([[0, 3, 1], [1, 3, 2], [0, 1, 3], [1, 2, 3]]).astype(np.int32)

        # Transform TCS to the DCS reference frame (RF).

        tcs_vertices = tcs_vertices_TCS @ R.T + contact_point
        tcs_normal = R @ tcs_normal_TCS[:,np.newaxis]
        d_tcs = tcs_vertices[np.newaxis,0,:] @ tcs_normal

        # The TCS normal must be oriented in opposite direction of the z-axis of the DCS RF.

        if tcs_normal[2] > -1e-10:
            valid = False
            z = 0.0
        else:
            intersection_vertices_in = self.intersection(tcs_vertices, tcs_normal, d_tcs, dcs_vertices_in, vision_tolerance)

            # Translation in z-direction.

            if intersection_vertices_in.shape[0] == 0:
                valid = False
                z = 0.0
            else:
                intersection_vertices_out = self.intersection(tcs_vertices, tcs_normal, d_tcs, dcs_vertices_out, -vision_tolerance)
                z = intersection_vertices_out[:,2].min()
                
                if visualization:
                    # 2D visualization.

                    tcs_vis = tcs_vertices[[0, 1, 3, 2, 0], :]
                    plt.plot(tcs_vis[:,0], tcs_vis[:,1], 'b')
                    dcs_vis = np.concatenate((dcs_vertices, np.expand_dims(dcs_vertices[0,:], 0)), axis=0)
                    plt.plot(dcs_vis[:,0], dcs_vis[:,1], 'g')
                    dcs_vis = np.concatenate((dcs_vertices_in, np.expand_dims(dcs_vertices_in[0,:], 0)), axis=0)
                    plt.plot(dcs_vis[:,0], dcs_vis[:,1], 'g:')
                    dcs_vis = np.concatenate((dcs_vertices_out, np.expand_dims(dcs_vertices_out[0,:], 0)), axis=0)
                    plt.plot(dcs_vis[:,0], dcs_vis[:,1], 'g:')
                    plt.plot(contact_point[0], contact_point[1], '+r')
                    plt.plot(intersection_vertices_in[:,0], intersection_vertices_in[:,1], 'yx')
                    plt.plot(intersection_vertices_out[:,0], intersection_vertices_out[:,1], 'yx')
                    ax = plt.gca()
                    ax.set_aspect('equal')
                    plt.show()

                    # 3D visualization.

                    z_shift = np.array([0.0, 0.0, z])
                    tcs_mesh = o3d.geometry.TriangleMesh()
                    tcs_mesh.vertices = o3d.utility.Vector3dVector(tcs_vertices - z_shift)
                    tcs_mesh.triangles = o3d.utility.Vector3iVector(tcs_triangles)
                    tcs_mesh.compute_vertex_normals()
                    tcs_mesh.paint_uniform_color([0.0, 0.5, 0.5])

                    dcs_mesh = o3d.geometry.TriangleMesh()
                    dcs_mesh.vertices = o3d.utility.Vector3dVector(dcs_vertices)
                    dcs_mesh.triangles = o3d.utility.Vector3iVector(dcs_triangles)
                    dcs_mesh.compute_vertex_normals()

                    dcs_normal = o3d.geometry.LineSet()
                    dcs_normal.points = o3d.utility.Vector3dVector(np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.07]]))
                    dcs_normal.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
                    dcs_normal.paint_uniform_color([0.0, 0.5, 0.0])

                    intersection_vertices_pcd = o3d.geometry.PointCloud()
                    intersection_vertices_pcd.points = o3d.utility.Vector3dVector(intersection_vertices_in - z_shift)
                    intersection_vertices_pcd.paint_uniform_color([1.0, 0.0, 0.0])

                    o3d.visualization.draw_geometries([tcs_mesh, dcs_mesh, dcs_normal, intersection_vertices_pcd])

                valid = True
        
        return valid, z

    def valid_contact_poses(self, tool_finger_distances, sphere_to_TCS_distance, vision_tolerance, contact_points, num_viewpoints = 300, num_rot_angles = 12):
        # Constants.

        num_contact_points = contact_points.shape[0]
        num_orientations = num_viewpoints * num_rot_angles
        num_samples = num_orientations * num_contact_points

        # Generate random orientataions.
 
        rot_mx = random_orientations(num_viewpoints * num_contact_points, num_rot_angles)

        # Only for debugging purpose!!!

        #rot_mx[22 * num_orientations, :, :] = np.array([[0, 1, 0], [-0.479426, 0, 0.877583], [0.877583, 0, 0.479426]])

        # Compute tool poses suitable for pushing the contact surface.

        valid_contact_poses_ = []
        #valid_poses = 0
        #invalid_poses = 0
        prev_contact_point = contact_points[0,:]
        for contact_point_idx in range(num_contact_points):  
            # if contact_point_idx == 22:
            #     debug = 0
            contact_point = contact_points[contact_point_idx,:]
            if contact_point[1] != prev_contact_point[1]:
                print(' ')
            print('.', end=' ')
            prev_contact_point = contact_point
            for orientation_idx in range(num_orientations):
                R = rot_mx[contact_point_idx * num_orientations + orientation_idx,:,:]
                valid, z = self.z_shift(self.tool.tool_contact_surface_params[:2,:], vision_tolerance, R, contact_point, visualization=False)
                if not valid:
                    valid, z = self.z_shift(self.tool.tool_contact_surface_params[1:,:], vision_tolerance, R, contact_point, visualization=False)
                if valid:
                    T_G_DD = self.tool_pose(R, contact_point, z, tool_finger_distances, sphere_to_TCS_distance)																					   
                    valid_contact_poses_.append(T_G_DD)
                    # valid_poses += 1
                    # print(1)
                # else:
                #     invalid_poses += 1
                #     print(0)
        print(' ')

        # print('valid:', valid_poses, 'invalid:', invalid_poses)
        return np.array(valid_contact_poses_)

    def collision_detection(self, tool_poses, vn):
        num_tool_poses = tool_poses.shape[0]
        num_tool_sample_spheres = self.tool.tool_sample_spheres.shape[0]
        c_G = rvl.homogeneous(self.tool.tool_sample_spheres[:,:3])
        c = tool_poses[:,np.newaxis,:,:] @ c_G[:,:,np.newaxis]
        y = vn.sdf(np.reshape(c, (num_tool_poses * num_tool_sample_spheres, 4))[:,:3])
        y = np.reshape(y, (num_tool_poses, num_tool_sample_spheres))
        e = (y - self.tool.tool_sample_spheres[:,3]).min(axis=1)
        return (e <= 0.0)
    
    def tool_pose(self, R, contact_point, z, tool_finger_distances, sphere_to_TCS_distance):
        T_G_TCS = np.eye(4)
        T_G_TCS[:3,3] = tool_finger_distances.copy() # define all coords for 3finger
        T_TCS_DD = np.eye(4)
        T_TCS_DD[:3,:3] = R
        T_TCS_DD[:3,3] = contact_point
        T_TCS_DD[2,3] -= z
        T_TCS_DD[2,3] += sphere_to_TCS_distance
        T_G_DD = T_TCS_DD @ T_G_TCS      
        c_G = rvl.homogeneous(self.tool.tool_sample_spheres[self.tool.tool_sample_spheres_contact,:3])
        c_DD = T_G_DD @ c_G[:,:,np.newaxis]
        e = (c_DD[:,2] - self.tool.tool_sample_spheres[self.tool.tool_sample_spheres_contact,3]).min()
        if e < 0.0:
            T_G_DD[2,3] -= e
        return T_G_DD

    def path(self, push_poses, init_pose):
        # Parameters.

        num_iterations = 3
        num_samples_per_iteration = 1000
        num_perturbations_per_sample = 100
        perturbation_dist = 0.02
        perturbation_angle_deg = 20.0

        # Constants.

        perturbation_angle_rad = np.deg2rad(perturbation_angle_deg)
        num_tool_sample_spheres = self.tool.tool_sample_spheres.shape[0]
        total_num_sample_spheres = num_tool_sample_spheres * num_samples_per_iteration

        # Init sample sphere centers.

        c_G = rvl.homogeneous(self.tool.tool_sample_spheres[:,:3])[:,:,np.newaxis]
        c_W_init = init_pose @ c_G
        c_W_init = np.tile(c_W_init, (num_samples_per_iteration,1))
        r = self.tool.tool_sample_spheres[:,3]
        r = np.tile(r, num_samples_per_iteration)

        # Iterations.

        samples = []
        for iter in range(num_iterations):
            # Random sampling.

            sample_idx = np.random.permutation(push_poses.shape[0])[:num_samples_per_iteration]
            new_samples = push_poses[sample_idx]
            samples.append(new_samples)

            # Collision detection.

            c_W = new_samples[:,np.newaxis,:,:] @ c_G
            c_W = np.reshape(c_W, (total_num_sample_spheres, 4, 1))
            collision_lines = np.concatenate((c_W[:,:,0], c_W_init[:,:,0]), axis=1)
            obstacles = self.dd.vn_env.line_obstacle(collision_lines, r)
            obstacles = np.reshape(obstacles, (num_samples_per_iteration, num_tool_sample_spheres, 2))



class door_model():
    def __init__(self):
        self.dd_contact_surface_params = np.array([0.1, 0.1])
        #self.dd_contact_surface_params = np.array([0.05, 0.01])
        self.dd_plate_params = np.array([0.3, 0.5, 0.018])
        self.dd_moving_to_static_part_distance = 0.005
        self.dd_static_side_width = 0.018
        self.dd_static_depth = 0.3
        self.dd_axis_distance = 0.01
        self.dd_opening_direction = 1.0

    def create(self, dd_state_deg, vision_tolerance):
        # Moving part pose with respect to the static part.

        T_A_W = np.eye(4)
        T_A_W[0,3] = self.dd_static_side_width
        T_A_W[1,3] = self.dd_static_side_width + self.dd_moving_to_static_part_distance + 0.5 * self.dd_plate_params[1]
        T_A_W[2,3] = 0.5 * self.dd_plate_params[2]  
        T_Arot_A = rvl.roty(np.deg2rad(dd_state_deg))
        T_Arot_DD = np.eye(4)
        T_Arot_DD[0,3] = self.dd_plate_params[0] + self.dd_moving_to_static_part_distance - self.dd_axis_distance
        T_Arot_DD[1,3] = 0.5 * self.dd_plate_params[1]
        T_Arot_DD[2,3] = -0.5 * self.dd_plate_params[2]
        if self.dd_opening_direction > 0.0:
            T_A_W[0,3] += (self.dd_moving_to_static_part_distance + self.dd_plate_params[0] + self.dd_moving_to_static_part_distance - self.dd_axis_distance)
        else:
            T_A_W[0,3] += self.dd_axis_distance
            T_Arot_DD[0,3] = -T_Arot_DD[0,3]
        self.T_DD_W = T_A_W @ T_Arot_A @ rvl.inv_transf(T_Arot_DD)

        # Door/drawer VN model.

        self.vn_dd = vn.vn()
        A = self.vn_dd.create_base_18()    
        dd_plate_max_vertex_coordinates = self.dd_plate_params.copy()
        dd_plate_max_vertex_coordinates[:2] += (2.0 * vision_tolerance)
        dd_plate_max_vertex_coordinates[2] = -dd_plate_max_vertex_coordinates[2]
        if self.dd_opening_direction < 0.0:
            dd_plate_max_vertex_coordinates[0] = -dd_plate_max_vertex_coordinates[0]
        dd_plate_box = (self.vn_dd.unit_box_vertices() + 1.0) * 0.5 * dd_plate_max_vertex_coordinates        
        if self.dd_opening_direction > 0.0:
            dd_plate_box[:,0] -= vision_tolerance
        else:
            dd_plate_box[:,0] += vision_tolerance
        dd_plate_box[:,1] -= vision_tolerance
        d_mov = self.vn_dd.convex_hull(A, dd_plate_box)
        self.vn_dd.add_bl_nodes(A, d_mov)
        self.vn_dd.add_hl_node(1.0, range(18))

        # Environment VN model.

        self.vn_env = vn.vn()
        self.vn_env.add_bl_nodes(A, d_mov)
        A_storage_space = np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]])
        dd_storage_space_max_vertex_coordinates = np.zeros(3)
        dd_storage_space_max_vertex_coordinates[0] = self.dd_plate_params[0] + 2.0 * self.dd_moving_to_static_part_distance
        dd_storage_space_max_vertex_coordinates[1] = self.dd_plate_params[1] + 2.0 * self.dd_moving_to_static_part_distance
        dd_storage_space_max_vertex_coordinates[2] = self.dd_static_depth
        dd_storage_space = (self.vn_env.unit_box_vertices() + 1.0) * 0.5 * dd_storage_space_max_vertex_coordinates
        dd_storage_space[:,0] += self.dd_static_side_width
        dd_storage_space[:,1] += self.dd_static_side_width
        d_storage_space = self.vn_env.concave_hull(A_storage_space, dd_storage_space)
        self.vn_env.add_bl_nodes(A_storage_space, d_storage_space)
        A_front = np.array([[0, 0, -1]])
        dd_front = self.vn_env.convex_hull(A_front, dd_storage_space)
        self.vn_env.add_bl_nodes(A_front, dd_front)
        self.vn_env.add_hl_node(1.0, range(18))
        self.vn_env.add_hl_node(-1.0, range(18,22))
        self.vn_env.add_hl_node(1.0, [22, 24])
        self.vn_env.add_hl_node(-1.0, [23, 25])
        self.vn_env.transform_bl_nodes(range(18), self.T_DD_W)                

        return T_A_W, T_Arot_DD

    def create2(self, dd_state_deg, vision_tolerance):
        # Moving part pose with respect to the static part.

        T_A_W = np.eye(4)
        # T_A_W[0,3] = self.dd_static_side_width + 2.0 * self.dd_moving_to_static_part_distance + self.dd_plate_params[0] - self.dd_axis_distance
        T_A_W[0,3] = self.dd_static_side_width + self.dd_moving_to_static_part_distance + self.dd_axis_distance/2
        T_A_W[1,3] = self.dd_static_side_width + self.dd_moving_to_static_part_distance + 0.5 * self.dd_plate_params[1]
        T_A_W[2,3] = 0.5 * self.dd_plate_params[2]
        T_Arot_A = rvl.roty(np.deg2rad(dd_state_deg))
        T_Arot_DD = np.eye(4)
        # T_Arot_DD[0,3] = self.dd_plate_params[0] + self.dd_moving_to_static_part_distance - self.dd_axis_distance
        T_Arot_DD[0,3] = -self.dd_plate_params[0] - self.dd_moving_to_static_part_distance + self.dd_axis_distance
        T_Arot_DD[1,3] = 0.5 * self.dd_plate_params[1]
        T_Arot_DD[2,3] = -0.5 * self.dd_plate_params[2]
        self.T_DD_W = T_A_W @ T_Arot_A @ rvl.inv_transf(T_Arot_DD)

        # Door/drawer VN model.

        self.vn_dd = vn.vn()
        A = self.vn_dd.create_base_18()    
        dd_plate_max_vertex_coordinates = self.dd_plate_params.copy()
        dd_plate_max_vertex_coordinates[:2] += (2.0 * vision_tolerance)
        dd_plate_max_vertex_coordinates[2] = -dd_plate_max_vertex_coordinates[2]
        dd_plate_box = (self.vn_dd.unit_box_vertices() + 1.0) * 0.5 * dd_plate_max_vertex_coordinates
        dd_plate_box[:,:2] -= vision_tolerance
        d_mov = self.vn_dd.convex_hull(A, dd_plate_box)
        self.vn_dd.add_bl_nodes(A, d_mov)
        self.vn_dd.add_hl_node(1.0, range(18))

        # Environment VN model.

        self.vn_env = vn.vn()
        self.vn_env.add_bl_nodes(A, d_mov)
        A_storage_space = np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]])
        dd_storage_space_max_vertex_coordinates = np.zeros(3)
        dd_storage_space_max_vertex_coordinates[0] = self.dd_plate_params[0] + 2.0 * self.dd_moving_to_static_part_distance
        dd_storage_space_max_vertex_coordinates[1] = self.dd_plate_params[1] + 2.0 * self.dd_moving_to_static_part_distance
        dd_storage_space_max_vertex_coordinates[2] = self.dd_static_depth
        dd_storage_space = (self.vn_env.unit_box_vertices() + 1.0) * 0.5 * dd_storage_space_max_vertex_coordinates
        dd_storage_space[:,0] += self.dd_static_side_width
        dd_storage_space[:,1] += self.dd_static_side_width
        d_storage_space = self.vn_env.concave_hull(A_storage_space, dd_storage_space)
        self.vn_env.add_bl_nodes(A_storage_space, d_storage_space)
        A_front = np.array([[0, 0, -1]])
        dd_front = self.vn_env.convex_hull(A_front, dd_storage_space)
        self.vn_env.add_bl_nodes(A_front, dd_front)
        self.vn_env.add_hl_node(1.0, range(18))
        self.vn_env.add_hl_node(-1.0, range(18,22))
        self.vn_env.add_hl_node(1.0, [22, 24])
        self.vn_env.add_hl_node(-1.0, [23, 25])
        self.vn_env.transform_bl_nodes(range(18), self.T_DD_W)
        
        return T_A_W, T_Arot_DD


    def create_mesh(self):
        dd_plate_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_plate_params[0], height=self.dd_plate_params[1], depth=self.dd_plate_params[2])    
        if self.dd_opening_direction > 0.0:
            dd_plate_mesh.translate((0.0, 0.0, -self.dd_plate_params[2]))    
        else:
            dd_plate_mesh.translate((-self.dd_plate_params[0], 0.0, -self.dd_plate_params[2]))    
        dd_plate_mesh.transform(self.T_DD_W)
        dd_plate_mesh.compute_vertex_normals()
        dd_plate_mesh.paint_uniform_color([0.8, 0.8, 0.8])
        dd_plate_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.05)
        dd_plate_rf.transform(self.T_DD_W)

        dd_static_top_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_plate_params[0] + 2.0 * (self.dd_moving_to_static_part_distance + self.dd_static_side_width),
            height=self.dd_static_side_width, depth=self.dd_static_depth)
        dd_static_bottom_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_plate_params[0] + 2.0 * (self.dd_moving_to_static_part_distance + self.dd_static_side_width),
            height=self.dd_static_side_width, depth=self.dd_static_depth)
        dd_static_bottom_mesh.translate((0.0, 2.0 * self.dd_moving_to_static_part_distance + self.dd_static_side_width + self.dd_plate_params[1], 0.0))
        dd_static_left_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_static_side_width, 
            height=self.dd_plate_params[1] + 2.0 * self.dd_moving_to_static_part_distance, depth=self.dd_static_depth)
        dd_static_left_mesh.translate((2.0 * self.dd_moving_to_static_part_distance + self.dd_static_side_width + self.dd_plate_params[0], 
            self.dd_static_side_width, 0.0))
        dd_static_right_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_static_side_width, 
            height=self.dd_plate_params[1] + 2.0 * self.dd_moving_to_static_part_distance, depth=self.dd_static_depth)
        dd_static_right_mesh.translate((0.0, self.dd_static_side_width, 0.0))
        dd_static_mesh = dd_static_top_mesh + dd_static_bottom_mesh + dd_static_left_mesh + dd_static_right_mesh
        #dd_static_mesh.translate((-dd_moving_to_static_part_distance - dd_static_side_width,
        #    -dd_moving_to_static_part_distance - dd_static_side_width, -dd_plate_params[2]))
        dd_static_mesh.compute_vertex_normals()
        dd_static_mesh.paint_uniform_color([0.8, 0.8, 0.8])
        
        
        dd_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.05)

        dd_mesh = dd_plate_mesh + dd_static_mesh + dd_plate_rf + dd_origin

        return dd_mesh

    def create_mesh2(self):
        dd_plate_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_plate_params[0], height=self.dd_plate_params[1], depth=self.dd_plate_params[2])    
        dd_plate_mesh.translate((-self.dd_plate_params[0], 0.0, -self.dd_plate_params[2]))
        dd_plate_mesh.transform(self.T_DD_W)
        dd_plate_mesh.compute_vertex_normals()
        dd_plate_mesh.paint_uniform_color([0.8, 0.8, 0.8])
        dd_plate_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.05)
        dd_plate_rf.transform(self.T_DD_W)

        dd_static_top_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_plate_params[0] + 2.0 * (self.dd_moving_to_static_part_distance + self.dd_static_side_width),
            height=self.dd_static_side_width, depth=self.dd_static_depth)
        dd_static_bottom_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_plate_params[0] + 2.0 * (self.dd_moving_to_static_part_distance + self.dd_static_side_width),
            height=self.dd_static_side_width, depth=self.dd_static_depth)
        dd_static_bottom_mesh.translate((0.0, 2.0 * self.dd_moving_to_static_part_distance + self.dd_static_side_width + self.dd_plate_params[1], 0.0))
        dd_static_left_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_static_side_width, 
            height=self.dd_plate_params[1] + 2.0 * self.dd_moving_to_static_part_distance, depth=self.dd_static_depth)
        dd_static_left_mesh.translate((2.0 * self.dd_moving_to_static_part_distance + self.dd_static_side_width + self.dd_plate_params[0], 
            self.dd_static_side_width, 0.0))
        dd_static_right_mesh = o3d.geometry.TriangleMesh.create_box(width=self.dd_static_side_width, 
            height=self.dd_plate_params[1] + 2.0 * self.dd_moving_to_static_part_distance, depth=self.dd_static_depth)
        dd_static_right_mesh.translate((0.0, self.dd_static_side_width, 0.0))
        dd_static_mesh = dd_static_top_mesh + dd_static_bottom_mesh + dd_static_left_mesh + dd_static_right_mesh
        #dd_static_mesh.translate((-dd_moving_to_static_part_distance - dd_static_side_width,
        #    -dd_moving_to_static_part_distance - dd_static_side_width, -dd_plate_params[2]))
        dd_static_mesh.compute_vertex_normals()
        dd_static_mesh.paint_uniform_color([0.8, 0.8, 0.8])

        dd_mesh = dd_plate_mesh + dd_static_mesh + dd_plate_rf

        return dd_mesh


class tool_model():
    def __init__(self, gripper_params):
	
        self.default_used = gripper_params['is_default_gripper']
        self.custom_gripper_spheres_path = gripper_params['custom_gripper_spheres_path']
        self.custom_gripper_model_path = gripper_params['custom_gripper_model_path']

        # Default gripper parameters	
        self.tool_finger_size = np.array([0.02, 0.02, 0.06])
        self.tool_palm_size = np.array([0.1, 0.02, 0.02])
		
        # Trapezoid points on the fingertips of the gripper w.r.t. the midpoint of the upper base of the trapezoid
        # self.tool_contact_surface_params = np.array([[0.0, 0.02, 0.0], [0.0, 0.03, -0.04]])
        # self.tool_contact_surface_params_default = np.array([[0.0, 0.01, 0.0], [0.0, 0.01, -0.02]])
        # self.tool_contact_surface_params_3finger = np.array([[0.0, -0.026, 0.0], [0.0, -0.031, -0.025]]) 
        # self.tool_contact_surface_params = self.tool_contact_surface_params_default if self.default_used else self.tool_contact_surface_params_3finger
        self.tool_contact_surface_params = np.array(gripper_params['tool_contact_surface_params'])


        # Distances from TCS to G in TCS frame
        # self.tool_finger_distances_default = [0.06/2., 0., 0.] # x, y, z
        # self.tool_finger_distances_3finger = [-0.155/2., 0., -0.102] # x, y, z
        # self.tool_finger_distances = self.tool_finger_distances_default.copy() if self.default_used else self.tool_finger_distances_3finger.copy()
        self.tool_finger_distances = np.array(gripper_params['tool_finger_distances'])
        
        # Largest distance between inspheres/exspheres 
        # self.sphere_to_TCS_distance_default = 0.
        # self.sphere_to_TCS_distance_3finger = 0.004609
        # self.sphere_to_TCS_distance = self.sphere_to_TCS_distance_default if self.default_used else self.sphere_to_TCS_distance_3finger
        self.sphere_to_TCS_distance = gripper_params['sphere_to_TCS_distance']
        
        # Distance between tool contact surfaces of the opposite fingers
        self.tool_finger_distance_default = 0.06
        self.tool_finger_distance_3finger = 0.155
        # self.tool_finger_distance = self.tool_finger_distance_default if self.default_used else self.tool_contact_surface_params_3finger
        self.tool_finger_distance = self.tool_finger_distance_default

    def create(self):
        # Tool sample spheres.

        if self.default_used:
            tool_sample_sphere_r = 0.5 * self.tool_finger_size[0]
            self.tool_sample_spheres = np.array([[-0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -tool_sample_sphere_r, tool_sample_sphere_r],
                                            [-0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -3.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [-0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -5.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [-0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -7.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [ 0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -tool_sample_sphere_r, tool_sample_sphere_r],
                                            [ 0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -3.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [ 0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -5.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [ 0.5 * (self.tool_finger_distance + self.tool_finger_size[0]), 0.0, -7.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [-2.0 * tool_sample_sphere_r, 0.0, -7.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [0.0, 0.0, -7.0 * tool_sample_sphere_r, tool_sample_sphere_r],
                                            [2.0 * tool_sample_sphere_r, 0.0, -7.0 * tool_sample_sphere_r, tool_sample_sphere_r]])        
        else:
            with open(self.custom_gripper_spheres_path, 'rb') as f:
                self.tool_sample_spheres = np.array(np.load(f))
            self.tool_sample_spheres_contact = np.arange(17,23)
            
            self.tool_sample_spheres /= 1000.


    def create_mesh(self, tool_color):
        if self.default_used: 
            finger = o3d.geometry.TriangleMesh.create_box(width=self.tool_finger_size[0], height=self.tool_finger_size[1], depth=self.tool_finger_size[2])
            finger1 = copy.deepcopy(finger).translate((0.5 * self.tool_finger_distance, -0.5 * self.tool_finger_size[1], -self.tool_finger_size[2]))
            finger2 = copy.deepcopy(finger).translate((-0.5 * self.tool_finger_distance - self.tool_finger_size[0], -0.5 * self.tool_finger_size[1], -self.tool_finger_size[2]))
            palm = o3d.geometry.TriangleMesh.create_box(width=self.tool_palm_size[0], height=self.tool_palm_size[1], depth=self.tool_palm_size[2])
            palm = palm.translate((-0.5 * self.tool_palm_size[0], -0.5 * self.tool_palm_size[1], -self.tool_finger_size[2] - self.tool_palm_size[2]))
            tool_mesh = finger1 + finger2 + palm
            tool_mesh.compute_vertex_normals()
            tool_mesh.paint_uniform_color(tool_color)
            tool_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 0.05)
            tool_mesh += tool_rf
        else:
            tool_mesh = o3d.io.read_triangle_mesh(self.custom_gripper_model_path)
            tool_mesh.paint_uniform_color(tool_color)

        return tool_mesh


def visualize_push(collision, door, tool, T_G_DD):
    if collision:
        tool_color = [1.0, 0.0, 0.0]
    else:
        tool_color = [0.0, 0.5, 0.5]
    # tool_mesh = tool.create_mesh(tool_color)
    tool_mesh = tool.create_mesh(tool_color)
    T_G_W = door.T_DD_W @ T_G_DD
    tool_mesh.transform(T_G_W)
    tool_mesh_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(tool_mesh)

    tool_sampling_sphere_centers_pcd = o3d.geometry.PointCloud()
    c_G = rvl.homogeneous(tool.tool_sample_spheres[:,:3])
    c_DD = c_G @ T_G_DD.T
    c_W = c_DD @ door.T_DD_W.T
    tool_sampling_sphere_centers_pcd.points = o3d.utility.Vector3dVector(c_W[:,:3])
    tool_sampling_sphere_centers_pcd.paint_uniform_color(tool_color)
 
    dd_mesh = door.create_mesh()

    return dd_mesh, tool_mesh, tool_mesh_wireframe, tool_sampling_sphere_centers_pcd


def visualize_push2(collision, door, tool, T_G_DD):
    if collision:
        tool_color = [1.0, 0.0, 0.0]
    else:
        tool_color = [0.0, 0.5, 0.5]
    # tool_mesh = tool.create_mesh(tool_color)
    tool_mesh = tool.create_mesh(tool_color)
    T_G_W = door.T_DD_W @ T_G_DD
    tool_mesh.transform(T_G_W)
    tool_mesh_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(tool_mesh)

    tool_sampling_sphere_centers_pcd = o3d.geometry.PointCloud()
    c_G = rvl.homogeneous(tool.tool_sample_spheres[:,:3])
    c_DD = c_G @ T_G_DD.T
    c_W = c_DD @ door.T_DD_W.T
    tool_sampling_sphere_centers_pcd.points = o3d.utility.Vector3dVector(c_W[:,:3])
    tool_sampling_sphere_centers_pcd.paint_uniform_color(tool_color)
 
    dd_mesh = door.create_mesh2()

    return dd_mesh, tool_mesh, tool_mesh_wireframe, tool_sampling_sphere_centers_pcd
 
 
def demo_single_random():
    # Parameters.
  
    dd_state_deg = -12.0
    contact_point = np.array([0.01, 0.01, 0.0])
    load_tool_pose = False
    is_3finger_used = True
    vision_tolerance = 0.005

    # Door model.

    door = door_model()
    door.create(dd_state_deg, vision_tolerance)

    # Tool model.

    tool = tool_model()
    # tool.create()
    tool.create_3finger() if is_3finger_used else tool.create()
    
    # Load tool pose.

    if load_tool_pose:
        T_G_DD = np.load("tool_pose.npy")
        valid = True
    else:
        # Compute z-shift.

        push_ = push(door, tool)
        valid = False
        while not valid:
            # Random orientation.

            rot_axis = 2.0 * np.random.rand(3) - 1.0
            rot_axis /= np.linalg.norm(rot_axis)
            rot_angle = np.random.rand() * np.pi
            R = rvl.angle_axis_to_rotmx(rot_axis, rot_angle)
            R = R[:3,:3]

            # Z-shift.

            valid, z = push_.z_shift(tool.tool_contact_surface_params[:2,:], vision_tolerance, R, contact_point, visualization=False)

        # Tool pose.

        T_G_DD = push_.tool_pose(R, contact_point, z, tool.tool_finger_distances, tool.sphere_to_TCS_distance)

    # Collision detection.

    c_G = rvl.homogeneous(tool.tool_sample_spheres[:,:3])
    c_DD = c_G @ T_G_DD.T
    c_W = c_DD @ door.T_DD_W.T
    y = door.vn_env.sdf(c_W[:,:3])    
    collision = ((y - tool.tool_sample_spheres[:,3]).min() <= 0.0)

    # VN line free space test.

    dd_static_front_size = np.array([door.dd_plate_params[0] + 2.0 * (door.dd_static_side_width + door.dd_moving_to_static_part_distance), 
        door.dd_plate_params[1] + 2.0 * (door.dd_static_side_width + door.dd_moving_to_static_part_distance)])
    x = np.linspace(-0.05, dd_static_front_size[0] + 0.05, 103)
    y = np.linspace(-0.05, dd_static_front_size[1] + 0.05, 103)
    z = np.linspace(-0.10, door.dd_static_depth + 0.05, 103)    
    # vn_grid_x, vn_grid_y = np.meshgrid(x, y)
    # vn_grid = np.stack((vn_grid_x, vn_grid_y, np.ones(vn_grid_x.shape)), axis=-1)
    # vn_grid = np.reshape(vn_grid, (vn_grid.shape[0] * vn_grid.shape[1], 3))
    # sample_lines = np.tile(vn_grid,2)
    # sample_lines[:,2] *= -1.0
    vn_grid_x, vn_grid_z = np.meshgrid(x, z)
    vn_grid = np.stack((vn_grid_x, 0.1 * np.ones(vn_grid_x.shape), vn_grid_z), axis=-1)
    vn_grid = np.reshape(vn_grid, (vn_grid.shape[0] * vn_grid.shape[1], 3))
    sample_lines = np.tile(vn_grid,2)
    sample_lines[:,1] = 1.0
    line_obstacle = door.vn_env.line_obstacle(sample_lines, np.zeros(sample_lines.shape[0]))
    
    # Save tool pose.

    if not load_tool_pose:
        np.save("tool_pose", T_G_DD)

    # Visualization.

    dd_mesh, tool_mesh, tool_mesh_wireframe, tool_sampling_sphere_centers_pcd = visualize_push(collision, door, tool, T_G_DD, is_3finger_used)

    vn_pcd = o3d.geometry.PointCloud()
    surface_points_in = np.logical_and(line_obstacle[:,0] >= 0.0, line_obstacle[:,0] <= 1.0)
    surface_points_out = np.logical_and(line_obstacle[:,1] >= 0.0, line_obstacle[:,1] <= 1.0)
    s_in = line_obstacle[surface_points_in,0][:,np.newaxis]
    s_out = line_obstacle[surface_points_out,1][:,np.newaxis]
    sample_line_endpoint1 = sample_lines[:,:3]
    sample_line_endpoint2 = sample_lines[:,3:]
    vn_dd_surface_points_in = (1.0 - s_in) * sample_line_endpoint1[surface_points_in,:] + s_in * sample_line_endpoint2[surface_points_in,:]
    vn_dd_surface_points_out = (1.0 - s_out) * sample_line_endpoint1[surface_points_out,:] + s_out * sample_line_endpoint2[surface_points_out,:]
    vn_dd_surface_points = np.concatenate((vn_dd_surface_points_in, vn_dd_surface_points_out), axis=0)
    vn_pcd.points = o3d.utility.Vector3dVector(vn_dd_surface_points)
    vn_pcd.paint_uniform_color([0.0, 1.0, 0.0])    

    # o3d.visualization.draw_geometries([tool_mesh_wireframe, tool_sampling_sphere_centers_pcd, dd_plate_mesh, dd_plate_rf, dd_static_mesh, vn_pcd])
    o3d.visualization.draw_geometries([tool_mesh_wireframe, tool_sampling_sphere_centers_pcd, dd_mesh])


def demo():
    # Parameters.

    #tool_contact_surface_params = np.array([[0.0, 0.02, 0.0], [0.0, 0.03, -0.04]])
    tool_contact_surface_params = np.array([[0.0, 0.01, 0.0], [0.0, 0.01, -0.02]])
    dd_contact_surface_params = np.array([0.1, 0.1])
    dd_contact_surface_sampling_resolution = 0.005
    num_viewpoints = 300
    num_rot_angles = 12

    # Generate viewpoints by sampling the unit sphere.

    cube_samples = np.random.rand(num_viewpoints, 3) - 0.5
    sphere_samples = cube_samples / np.linalg.norm(cube_samples, axis=1)[:,np.newaxis]

    # Sphere sampling visualization.

    # unit_sphere_pcd = o3d.geometry.PointCloud()
    # unit_sphere_pcd.points = o3d.utility.Vector3dVector(sphere_samples)
    # unit_sphere_pcd.paint_uniform_color([0.0, 1.0, 0.0])

    # o3d.visualization.draw_geometries([unit_sphere_pcd])

    # Generate random orientations.

    rot_mx = np.zeros((num_viewpoints, 3, 3))
    rot_mx[:,:,2] = sphere_samples
    axis_idx = np.argmin(np.abs(sphere_samples), axis=1)
    v = np.zeros((num_viewpoints, 3))
    np.put_along_axis(v, axis_idx[:,np.newaxis], 1.0, axis=1)
    rot_mx[:,:,0] = np.cross(sphere_samples,v)
    rot_mx[:,:,0] = rot_mx[:,:,0] / np.linalg.norm(rot_mx[:,:,0], axis=1)[:,np.newaxis]
    rot_mx[:,:,1] = np.cross(rot_mx[:,:,2], rot_mx[:,:,0])

    # Feasible pushes.

    push_ = push(tool_contact_surface_params, dd_contact_surface_params)
    d_rot_angle = 2.0 * np.pi / num_rot_angles
    num_dd_contact_point_samples_x = dd_contact_surface_params[0] / dd_contact_surface_sampling_resolution
    num_dd_contact_point_samples_y = dd_contact_surface_params[1] / dd_contact_surface_sampling_resolution
    for view_idx in range(num_viewpoints):
        first_rot_angle = d_rot_angle * np.random.rand()
        for rot_angle_idx in range(num_rot_angles):
            rot_angle = first_rot_angle + rot_angle_idx * d_rot_angle
            R = rot_mx[view_idx,:,:] @ rvl.rotz(rot_angle)[:3,:3]
            for dd_contact_point_y_idx in range(num_dd_contact_point_samples_y):
                contact_point_y = dd_contact_point_y_idx * dd_contact_surface_sampling_resolution
                for dd_contact_point_x_idx in range(num_dd_contact_point_samples_x):
                    contact_point_x = dd_contact_point_x_idx * dd_contact_surface_sampling_resolution
                    contact_point = np.array([contact_point_x, contact_point_y, 0.0])
                    valid, z = push_.z_shift(tool_contact_surface_params, 0.01, R, contact_point, visualization=True)
                    if valid:
                        print('valid')
 
    print('completed')

def demo_vn():
    # Parameters.

    dd_plate_params = np.array([0.3, 0.5, 0.1])

    # Door/drawer VN model.

    vn_env = vn.vn()
    vn_env.create_base_18()
    dd_plate_box = (vn_env.unit_box_vertices() + 1.0) * 0.5 * dd_plate_params
    vn_env.d = vn_env.convex_hull(dd_plate_box)
    vn_env.add_hl_node(1.0, range(18))
    vn_env.add_hl_node(-1.0, [0, 1, 3, 4])

    # Visualization.

    #sdf = vn_env.sdf(np.array([[0, 0, 0]]))

    samples = 2.0 * dd_plate_params * np.random.rand(100000, 3) - 0.5 * dd_plate_params
    sdf = vn_env.sdf(samples)
    in_pcd = o3d.geometry.PointCloud()
    in_pcd.points = o3d.utility.Vector3dVector(samples[sdf <= 0, :])
    in_pcd.paint_uniform_color([0.0, 0.0, 1.0])
    out_pcd = o3d.geometry.PointCloud()
    out_pcd.points = o3d.utility.Vector3dVector(samples[sdf > 0, :])
    out_pcd.paint_uniform_color([0.0, 1.0, 0.0])    

    #o3d.visualization.draw_geometries([in_pcd, out_pcd])
    o3d.visualization.draw_geometries([in_pcd])

def demo_push_poses():
    # Parameters.
  
    dd_state_deg = 12.0
    num_viewpoints = 100
    num_rot_angles = 12
    load_valid_contact_poses_from_file = False
    load_feasible_poses_from_file = False
    contact_point_sampling_offset = 0.02
    use_default_gripper = False
    vision_tolerance = 0.007

    if use_default_gripper:
        custom_gripper_spheres_path = ''
        custom_gripper_model_path = ''
        tool_contact_surface_params = np.array([[0.0, 0.01, 0.0], [0.0, 0.01, -0.02], [-0.02, 0.01, 0.0]])
        # tool_finger_distances = [-0.155/2., 0., -0.102] # x, y, z
        tool_finger_distances = [0.06/2., 0., 0.] # x, y, z
        sphere_to_TCS_distance = 0.
    else:
        # Simundic
        custom_gripper_spheres_path = '/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/gripper_spheres.npy'
        custom_gripper_model_path = '/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.stl'
        # END: Simundic
        # custom_gripper_spheres_path = '3finger_gripper/gripper_spheres.npy'
        # custom_gripper_model_path = '3finger_gripper/robotiq_3f_gripper_simplified.stl'
        
        tool_contact_surface_params = np.array([[0.0, -0.026, 0.0], [0.0, -0.031, -0.025], [0.006, -0.026, 0.0]])
        tool_finger_distances = [-0.155/2., 0., -0.102] # x, y, z
        sphere_to_TCS_distance = 0.004609

    gripper_params = {'is_default_gripper': use_default_gripper,
                        'custom_gripper_spheres_path': custom_gripper_spheres_path, 
                        'custom_gripper_model_path': custom_gripper_model_path,
                        'tool_contact_surface_params': tool_contact_surface_params,
                        'tool_finger_distances': tool_finger_distances,
                        'sphere_to_TCS_distance': sphere_to_TCS_distance}

    # Door model.

    door = door_model()
    door.dd_opening_direction = -1.0
    door.create(dd_state_deg, vision_tolerance)
     
    # Tool model.

    tool = tool_model(gripper_params)
    tool.create()

    # Contact points.

    if door.dd_opening_direction > 0.0:
        x = np.linspace(-contact_point_sampling_offset, door.dd_contact_surface_params[0], 25)
    else:
        x = np.linspace(-door.dd_contact_surface_params[0], contact_point_sampling_offset, 25)
    y = np.linspace(-contact_point_sampling_offset, door.dd_contact_surface_params[1], 25)
    dd_grid_x, dd_grid_y = np.meshgrid(x, y)
    contact_points = np.stack((dd_grid_x, dd_grid_y, np.zeros(dd_grid_x.shape)), axis=-1)
    contact_points = np.reshape(contact_points, (contact_points.shape[0] * contact_points.shape[1], 3))

    # contact_points = np.array([[0.1, 0.01, 0.0]])

    # Push tool.

    push_ = push(door, tool)

    # T_G_DD = np.eye(4)
    # T_G_DD[0,3] = 0.105
    # T_G_DD[2,3] = 0.05
    # collision = push_.collision_detection(T_G_DD[np.newaxis,:,:], door.vn_dd)
    # dd_mesh, tool_mesh, tool_mesh_wireframe, tool_sampling_sphere_centers_pcd = visualize_push(collision[0], door, tool, T_G_DD)
    # o3d.visualization.draw_geometries([tool_mesh_wireframe, tool_sampling_sphere_centers_pcd, dd_mesh])

    # Valid contact poses.
        
    if load_valid_contact_poses_from_file:
        valid_contact_poses_ = np.load("valid_contact_poses.npy")
    else:        
        valid_contact_poses_ = push_.valid_contact_poses(tool.tool_finger_distances, tool.sphere_to_TCS_distance, vision_tolerance, contact_points, num_viewpoints=num_viewpoints, num_rot_angles=num_rot_angles)
        np.save("valid_contact_poses", valid_contact_poses_)

    # Feasible poses (no collision with the door/drawer plate).

    if load_feasible_poses_from_file:
        feasible_poses = np.load('feasible_poses.npy')
    else:
        collision = push_.collision_detection(valid_contact_poses_, door.vn_dd)
        feasible_poses = valid_contact_poses_[np.logical_not(collision),:]
        np.save('feasible_poses', feasible_poses)

    # Collision-free poses.

    T_G_W = door.T_DD_W @ feasible_poses
    collision = push_.collision_detection(T_G_W, door.vn_env)
    contact_free_poses = feasible_poses[np.logical_not(collision),:]

    # Visualization.

    samples = contact_free_poses
    # samples = feasible_poses
    collision_ = np.zeros(samples.shape[0]).astype('bool')
    for visualization_idx in range(10):
        print('sample', visualization_idx)
        good_sample = False
        while not good_sample:
            sample_idx = np.random.randint(samples.shape[0])
            # sample_idx = 0
            T_G_DD = samples[sample_idx,:,:]
            p_ref_G = np.ones(4)
            p_ref_G[:3] = -np.array([tool_finger_distances])
            p_ref_DD = T_G_DD @ p_ref_G
            good_sample = (p_ref_DD[0] < 0.0 or p_ref_DD[1] < 0.0)
        dd_mesh, tool_mesh, tool_mesh_wireframe, tool_sampling_sphere_centers_pcd = visualize_push(collision_[sample_idx], door, tool, T_G_DD)
        o3d.visualization.draw_geometries([tool_mesh_wireframe, tool_sampling_sphere_centers_pcd, dd_mesh])


def demo_push_poses_ros(door_dims: np.array,
                        static_depth: float,
                        dd_state_deg: float, 
                        num_viewpoints: int, 
                        num_rot_angles: int,
                        opening_direction: float=1.0,
                        load_valid_contact_poses_from_file: bool=False, 
                        load_feasible_poses_from_file: bool=False, 
                        gripper_params: dict={},
                        feasible_poses_path: str='',
                        contact_free_poses_path: str='',
                        valid_contact_poses_path: str='',
                        visualization: bool=False):
    # Parameters.
  
    # dd_state_deg = -12.0
    # num_viewpoints = 100
    # num_rot_angles = 12
    # load_valid_contact_poses_from_file = False
    # load_feasible_poses_from_file = False
    
    if gripper_params['is_default_gripper']:
        custom_gripper_spheres_path = ''
        custom_gripper_model_path = ''
        tool_contact_surface_params = np.array([[0.0, 0.01, 0.0], [0.0, 0.01, -0.02]])
        tool_finger_distances = [-0.155/2., 0., -0.102] # x, y, z
        tool_finger_distances = [0.06/2., 0., 0.] # x, y, z
        sphere_to_TCS_distance = 0.
    else:
        custom_gripper_spheres_path = '/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/gripper_spheres.npy'
        custom_gripper_model_path = '/home/RVLuser/rvl-linux/python/DDMan/3finger_gripper/robotiq_3f_gripper_simplified.stl'
        
        tool_contact_surface_params = np.array([[0.0, -0.026, 0.0], [0.0, -0.031, -0.025]])
        tool_finger_distances = [-0.155/2., 0., -0.102] # x, y, z
        sphere_to_TCS_distance = 0.004609
    
    contact_point_sampling_offset = 0.02
    vision_tolerance = 0.007
    
    # gripper_params = {'is_default_gripper': use_default_gripper,
    #                     'custom_gripper_spheres_path': custom_gripper_spheres_path, 
    #                     'custom_gripper_model_path': custom_gripper_model_path,
    #                     'tool_contact_surface_params': tool_contact_surface_params,
    #                     'tool_finger_distances': tool_finger_distances,
    #                     'sphere_to_TCS_distance': sphere_to_TCS_distance}
   
    # Door model.
    door = door_model()
    door.dd_plate_params = door_dims
    door.dd_static_depth = static_depth
    door.dd_opening_direction = -1
    _, _ = door.create(dd_state_deg, vision_tolerance)

    # Tool model.
    tool = tool_model(gripper_params)
    tool.create()

    # Contact points.
    if door.dd_opening_direction > 0.0:
        x = np.linspace(-contact_point_sampling_offset, door.dd_contact_surface_params[0], 25)
    else:
        x = np.linspace(-door.dd_contact_surface_params[0], contact_point_sampling_offset, 25)
    y = np.linspace(-contact_point_sampling_offset, door.dd_contact_surface_params[1], 25)
    dd_grid_x, dd_grid_y = np.meshgrid(x, y)
    contact_points = np.stack((dd_grid_x, dd_grid_y, np.zeros(dd_grid_x.shape)), axis=-1)
    contact_points = np.reshape(contact_points, (contact_points.shape[0] * contact_points.shape[1], 3))

    # contact_points = np.array([[0.1, 0.01, 0.0]])

    # Push tool.

    push_ = push(door, tool)

    # Valid contact poses.
        
    if load_valid_contact_poses_from_file:
        valid_contact_poses_ = np.load(valid_contact_poses_path)
    else:        
        # valid_contact_poses_ = push_.valid_contact_poses(tool.tool_finger_distance, contact_points, num_viewpoints=num_viewpoints, num_rot_angles=num_rot_angles)
        valid_contact_poses_ = push_.valid_contact_poses(tool.tool_finger_distances, tool.sphere_to_TCS_distance, vision_tolerance, contact_points, num_viewpoints=num_viewpoints, num_rot_angles=num_rot_angles)
        np.save(valid_contact_poses_path, valid_contact_poses_)

    # Feasible poses (no collision with the door/drawer plate).

    if load_feasible_poses_from_file:
        feasible_poses = np.load(feasible_poses_path)
    else:
        collision = push_.collision_detection(valid_contact_poses_, door.vn_dd)
        feasible_poses = valid_contact_poses_[np.logical_not(collision),:]
        np.save(feasible_poses_path, feasible_poses)

    # Collision-free poses.

    T_G_W = door.T_DD_W @ feasible_poses
    collision = push_.collision_detection(T_G_W, door.vn_env)
    contact_free_poses = feasible_poses[np.logical_not(collision),:]
    if not load_feasible_poses_from_file:
        np.save(contact_free_poses_path, contact_free_poses)

    # Visualization.
    if visualization:
        samples = contact_free_poses
        # samples = valid_contact_poses_
        collision_ = np.zeros(samples.shape[0]).astype('bool')
        for visualization_idx in range(2):
            print('sample', visualization_idx)
            # sample_idx = np.random.randint(samples.shape[0])
            sample_idx = visualization_idx
            # sample_idx = 0
            T_G_DD = samples[sample_idx,:,:]
            dd_mesh, tool_mesh, tool_mesh_wireframe, tool_sampling_sphere_centers_pcd = visualize_push(collision_[sample_idx], door, tool, T_G_DD)


            # Visualize gripper origin
            tool_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
            tool_origin.transform(door.T_DD_W @ T_G_DD)
    
            o3d.visualization.draw_geometries([tool_mesh_wireframe, tool_sampling_sphere_centers_pcd, dd_mesh, tool_origin])
    
    # return feasible_poses
    return contact_free_poses


def visualize_dd():
    # Door model.
    door = door_model()
    T_A_W, T_Arot_DD = door.create(-45)
    
    dd_mesh = door.create_mesh()
    
    origin_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    
    # T_Arot_DD_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    # T_Arot_DD_mesh.transform(T_Arot_DD)

    T_A_W_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    T_A_W_mesh.transform(T_A_W)

    # o3d.visualization.draw_geometries([dd_mesh, origin_mesh, T_Arot_DD_mesh, T_A_W_mesh])
    o3d.visualization.draw_geometries([dd_mesh, origin_mesh, T_A_W_mesh])

    
