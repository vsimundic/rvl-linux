#!/usr/bin/env python

import rospy
import csv
import os
from core.ur5_commander import UR5Commander
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from core.transforms  import matrix_to_pose, rot_z
from tf.transformations import quaternion_from_euler
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
import numpy as np
from gazebo_msgs.msg import ModelState
import tf
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from gazebo_push_open.cabinet_model import Cabinet
from core.util import read_config, read_csv_DataFrame
from urdf_parser_py.urdf import URDF
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from rospkg import RosPack

# Function to read joint values from a CSV file
def read_joint_values_from_csv(file_path) -> list:
    joint_values = []
    with open(file_path, mode='r') as csv_file:
        csv_reader = csv.reader(csv_file)
        headers = next(csv_reader)
        for row in csv_reader:
            # Convert each row to a list of floats
            joint_values.append([float(value) for value in row[-6:]])
    return joint_values

def spawn_model(model_name, model_path, T=np.ndarray, frame_id="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Read the URDF file
    with open(model_path, 'r') as file:
        model_xml = file.read()

    # Define the pose for the model
    pose = matrix_to_pose(T)

    try:
        # Call the spawn model service
        spawn_model_service(model_name=model_name, model_xml=model_xml, robot_namespace='', initial_pose=pose, reference_frame=frame_id)
        rospy.loginfo(f"Successfully spawned {model_name} at ({pose.position.x}, {pose.position.y}, {pose.position.z}) with reference frame '{frame_id}'")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model_name}: {e}")

def delete_gazebo_model(model_name):
    """Delete a model in Gazebo."""
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp = delete_model(model_name)
        rospy.loginfo(f"Model '{model_name}' deleted.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Delete Model service call failed: {e}")


def main():
    # Initialize the ROS node
    rospy.init_node('test_tsr_cabinet_open_node', anonymous=True)

    # Cabinet URDF path
    urdf_path = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.urdf'

    # Read door data from csv
    csv_path = '/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/simulation_results_real_robot_20240305_final.csv'
    data = read_csv_DataFrame(csv_path)
    # # doors = np.load('/home/RVLuser/ferit_ur5_ws/src/cosper/push_simulation/config/door_params_poses_exp3_axis_left.npy')
    # rows = data.shape[0]
    # data = data.loc[((data['path_found'] == True) & 
    #                         (data['traj_success'] == True) & 
    #                         (data['contact_free'] == True) & 
    #                         (data['door_opened'] == True))] 

    # doors = data.to_numpy()[:, 4:]
    rp = RosPack()
    pkg_path = rp.get_path('path_planning')

    door_configs_path = os.path.join(pkg_path, 'door_configurations_axis_left.npy')
    doors = np.load(door_configs_path)
    
    T_B_S = np.eye(4)
    T_B_S[2, 3] = 0.005
    
    # Doors from real experiments (first successful)
    # door_width,door_height,x,y,z,rot_z,axis_pos
    for i in range(7, doors.shape[0]):
        door = doors[i, :]
        print('\n')
        print(doors[i])
        print('\n')
        # door = [0.396,0.496,-0.23390909091957188,0.6468227769292677,0.257,2.453959030822901,-1.0]

        T_A_S = np.eye(4)
        T_A_S[:3, 3] = np.array(door[2:5])
        T_A_S[2, 3] += T_B_S[2, 3]
        T_A_S[1, 3] += 0.1

        Tz_init = np.eye(4)
        Tz_init[:3, :3] = rot_z(np.radians(90.))
        T_A_S = T_A_S @ Tz_init
        Tz = np.eye(4)
        Tz[:3, :3] = rot_z(np.radians(door[ 5]))
        T_A_S = T_A_S @ Tz
        axis_pos = door[-1]
        # Create a cabinet object
        cabinet_model = Cabinet(door_params=np.array([door[0], door[1], 0.018, 0.4]), 
                                axis_pos=axis_pos,
                                T_A_S=T_A_S,
                                save_path=urdf_path,
                                has_handle=True)
        cabinet_mesh_filename = '/home/RVLuser/ferit_ur5_ws/cabinet_handle_test.ply'
        cabinet_model.save_mesh(cabinet_mesh_filename)

        cabinet_model.delete_model_gazebo()
        cabinet_model.delete_model_gazebo_sphere()

        
        robot = UR5Commander()
        # robot.send_named_pose('up')

        # # Add cabinet to scene interface
        # robot.remove_from_scene('cabinet')
        # T_O_B = np.linalg.inv(robot.T_B_S) @ cabinet_model.T_O_S
        # robot.add_mesh_to_scene(cabinet_mesh_filename, 'cabinet', T_O_B)

        # Spawning model in Gazebo
        cabinet_model.spawn_model_gazebo()
        
        if True:
            # cabinet_model.set_door_state_gazebo(45.)
            T_H_S = cabinet_model.T_A_S @ cabinet_model.T_H_A
            print(cabinet_model.T_H_A)
            print(T_H_S)
            print(cabinet_model.T_O_S @ cabinet_model.T_A_O @ cabinet_model.T_H_A)
            cabinet_model.spawn_model_gazebo_sphere_T(T_H_S)

            # T_D_S = cabinet_model.T_A_S @ cabinet_model.T_D_A
            # T_A_S = cabinet_model.T_A_S
            # T_sp_A  = np.eye(4)
            # T_sp_A[0, 3] = 0.05
            # cabinet_model.spawn_model_gazebo_sphere_T(T_D_S, name='sphereD')
            pass
        


        ### TSR matrices definition ###
        T0_w = T_A_S
        T0_w_pose =  matrix_to_pose(T0_w)
        t0_w = T0_w_pose.position
        q0_w = T0_w_pose.orientation

        print("  T0_w:")
        print('    translation: [%f, %f, %f]' % (t0_w.x, t0_w.y, t0_w.z))
        print('    rotation: [%f, %f, %f, %f] #wxyz' % (q0_w.w, q0_w.x, q0_w.y, q0_w.z))

        T_6_H = np.eye(4)
        T_6_H[:3, :3] = np.array([[0, 0, -axis_pos],
                                  [axis_pos, 0, 0],
                                  [0, -1, 0]])

        Tw_e = cabinet_model.T_H_A.copy()
        Tw_e[:3, :3] = np.array([[0, 0, 1],
                                [-1, 0, 0],
                                [0, -1, 0]])
        Tw_e[0, 3] -= 0.28
        Tz = np.eye(4)
        Tz[:3,:3] = rot_z(np.radians(45.)) # rotate gripper 45 deg to align fingers with handle
        Tw_e = Tw_e @ Tz
        Tw_e_pose =  matrix_to_pose(Tw_e)
        tw_e = Tw_e_pose.position
        qw_e = Tw_e_pose.orientation
        print('  Tw_e:')
        print('    translation: [%f, %f, %f]' % (tw_e.x, tw_e.y, tw_e.z))
        print('    rotation: [%f, %f, %f, %f] #wxyz' % (qw_e.w, qw_e.x, qw_e.y, qw_e.z))


        # a = input('Any letter to continue, blank to abort: \n>>')
        # if a == '':
        #     print('No confirmation for trajectory execution. Aborting.')
        #     return


        T_T_A = Tw_e
        T_T_B_goal = np.linalg.inv(robot.T_B_S) @ cabinet_model.T_A_S @ T_T_A

        # T_G_T = np.eye(4)
        # T_G_T[2, 3] = 0.28
        # T_G_B = np.linalg.inv(T_B_S) @ T0_w @ Tw_e
        # T_T_B_goal = T_G_B @ np.linalg.inv(T_G_T)
        # print(T_T_B_goal)

        # Create an instance of UR5Commander

        q_goal = robot.get_inverse_kin(robot.get_current_joint_values(), T_T_B_goal)
        if q_goal is None:
            return
        print("initial_config: {}".format(q_goal))
        # robot.send_pose_to_robot(T_T_B_goal, wait=True)

        # T_T_B_current = robot.get_current_tool_pose()

        # dist = np.linalg.norm(T_T_B_current-T_T_B_goal)

        # if dist < 0.01:
        #     print("Found working example")
        #     print("Index: %d" % i)
        #     print("Door params: {}".format(door))
        #     print("Joint values {}".format(robot.get_current_joint_values()))
        #     # break

        Tz_90 = np.eye(4) 
        Tz_90[:3,:3] = rot_z(np.deg2rad(-45))
        T_A_S_rotated = T_A_S @ Tz_90
        T_T_B_goal_rotated = np.linalg.inv(robot.T_B_S) @ T_A_S_rotated @ T_T_A

        q_goal_rot = robot.get_inverse_kin(q_goal, T_T_B_goal_rotated)
        if q_goal_rot is None:
            print("Cannot rotate doors! Aborting.")
            return
        print("goal_config: {}".format(q_goal_rot))

        # Load joint values from CSV
        file_path = '/home/RVLuser/ferit_ur5_ws/saved_trajectory_cabinet_opening.csv'
        joint_trajectory = read_joint_values_from_csv(file_path)

        # Send each joint configuration to the UR5 robot

        robot.send_joint_values_to_robot(joint_trajectory[0]) # get in position for grasp
        rospy.sleep(1)


        # Attach handle to gripper
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        attach_srv.wait_for_service()

        rospy.loginfo('Attaching handle to gripper')
        req = AttachRequest()
        req.model_name_1 = 'robot'
        req.link_name_1 = 'wrist_3_link'
        req.model_name_2 = 'my_cabinet'
        req.link_name_2 = 'door_link'

        attach_srv.call(req)
        
        robot.send_multiple_joint_space_poses_to_robot(joint_trajectory, 15.)

        rospy.loginfo("Joint trajectory execution completed.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass