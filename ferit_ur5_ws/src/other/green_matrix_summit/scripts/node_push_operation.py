#!/usr/bin/env python

import numpy as np
import os
import sys
import rospkg
import json
from read_json import read_ao_dict_from_json
import tf
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import geometry_msgs.msg
import rospy
import moveit_commander
from read_json import read_ao_dict_from_json
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel
import open3d as o3d
from std_msgs.msg import Bool
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from generate_cabinet_urdf import generate_cabinet_urdf_from_door_panel
from geometry_msgs.msg import Pose

class PushOperator():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('push_operation_node', anonymous=True)

        self.push_sub = rospy.Subscriber('/push_operation', Bool, self.callback)
        self.push_pub = rospy.Publisher('/build_model', Bool, queue_size=1)

        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('ao_manipulation')

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = 'arm'
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.publish_msg = Bool()
        self.publish_msg.data = True

        rospy.spin()

    def spawn_model(self, w_door, h_door, TA0, model_name):
        model_xml, TXA = generate_cabinet_urdf_from_door_panel(w_door=w_door, 
                                                        h_door=h_door, 
                                                        d_door=0.018)

        # TA0 = np.load(os.path.join(self.pkg_path, 'config', 'TA0.npy'))
        TX0 = TA0 @ TXA
        
        init_pose = Pose()
        init_pose.position.x = TX0[0, 3]
        init_pose.position.y = TX0[1, 3]
        init_pose.position.z = TX0[2, 3]
        q = quaternion_from_matrix(TX0)
        init_pose.orientation.x = q[0]
        init_pose.orientation.y = q[1]
        init_pose.orientation.z = q[2]
        init_pose.orientation.w = q[3]

        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            _ = spawn_urdf_model_proxy(model_name, model_xml, '/', init_pose, 'world')
            rospy.loginfo('URDF model spawned successfully')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to spawn URDF model: %s' % e)


    def check_if_model_exists(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state_request = GetModelStateRequest()
        model_state_request.model_name = 'my_cabinet'
        model_state = get_model_state(model_state_request)
        return model_state.success


    def callback(self, push_msg):
        if push_msg:
            ao = read_ao_dict_from_json(os.path.join(self.pkg_path, 'config', 'ao.txt'))
            TA0 = np.load(os.path.join(self.pkg_path, 'config', 'TA0.npy'))
            w_door = ao['s'][0]
            h_door = ao['s'][1]
            d_door = 0.018


            if not self.check_if_model_exists():
                self.spawn_model(w_door, h_door, TA0, 'my_cabinet')

            TPA = np.eye(4)
            TPA[:3, 3] = np.array([d_door/2. + 0.001, w_door, h_door/2.-0.05])

            TP0 = TA0 @ TPA
            
            TGT = np.eye(4)
            TGT[:3, 3] = np.array([0., 0., 0.217])

            TGP = np.array([[0, 0, -1, 0],
                            [0, -1, 0, 0],
                            [-1, 0, 0, 0],
                            [0, 0, 0, 1]])
            
            # TTB = np.load(os.path.join(self.pkg_path, 'config', 'TTB_0.npy'))
            TB0 = np.load(os.path.join(self.pkg_path, 'config', 'TB0_0.npy'))

            # TPB = np.linalg.inv(TB0) @ TP0
            TT_B = np.linalg.inv(TB0) @ TP0 @ TGP @ np.linalg.inv(TGT)
            print(TT_B)
            TGoal = TT_B.copy()

            TGoal_prev = TGoal.copy()
            TGoal_prev[0, 3] -= 0.2

            self.send_pose_to_robot(TGoal_prev, wait=True)
            self.send_pose_to_robot(TGoal, wait=True, cartesian=True)
            rospy.sleep(1)

            self.send_pose_to_robot(TGoal_prev, wait=False, cartesian=True)

            angle_deg = -20
            angle_delta = np.linspace(0, angle_deg, num=100)
            angle_counter = 0
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                if angle_counter < 100:

                    if not self.check_if_model_exists():
                        rospy.sleep(2)
                        self.push_pub.publish(self.publish_msg)
                        return

                    angle_rad = np.radians(angle_delta[angle_counter])

                    c = np.cos(angle_rad)
                    s = np.sin(angle_rad)
                    Tz = np.array([[c, -s, 0, 0],
                                    [s, c, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
                    TP_0 = TA0 @ Tz @ TPA

                    tool_pose = self.group.get_current_pose().pose
                    t = np.array([tool_pose.position.x, tool_pose.position.y, tool_pose.position.z])
                    q = np.array([tool_pose.orientation.x, tool_pose.orientation.y, tool_pose.orientation.z, tool_pose.orientation.w])
                    TTnB = quaternion_matrix(q)
                    TTnB[:3, 3] = t
                    TTn0 = TB0 @ TTnB

                    if TTn0[0, 3] > TP_0[0, 3]:
                        continue
                    
                    success = self.move_model_joint('my_cabinet', 'joint_0', np.radians(angle_delta[angle_counter]))
                    if not success:
                        continue
                    angle_counter += 1
                else:
                    break
            
            # Calculate points where the gripper needs to go to be able to open the door
            angle_rad = np.radians(angle_deg)
            c = np.cos(angle_rad)
            s = np.sin(angle_rad)
            Tz = np.array([[c, -s, 0, 0],
                            [s, c, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
            TP_0 = TA0 @ Tz @ TPA

            RG_P = np.array([[0, 1, 0],
                            [0, 0, -1],
                            [-1, 0, 0]])
            # tG_P = np.array([0.13, 0.15, 0])
            TG_P = np.eye(4)
            TG_P[:3, :3] = RG_P.copy()
            # TG_P[:3, 3] = tG_P.copy()
            TT_B = np.linalg.inv(TB0) @ TP_0 @ TG_P @ np.linalg.inv(TGT)
            self.send_pose_to_robot(TT_B, wait=True, cartesian=True)

            TG_P2 = TG_P.copy()
            TG_P2[1, 3] = -0.03 
            TT_B = np.linalg.inv(TB0) @ TP_0 @ TG_P2 @ np.linalg.inv(TGT)
            self.send_pose_to_robot(TT_B, wait=True, cartesian=True)

            goal_poses = []
            angle_open_delta = np.linspace(angle_deg, -60, num=20)
            for da_deg in angle_open_delta:
                    angle_rad = np.radians(da_deg)

                    c = np.cos(angle_rad)
                    s = np.sin(angle_rad)
                    Tz = np.array([[c, -s, 0, 0],
                                    [s, c, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
                    TP_0 = TA0 @ Tz @ TPA  
                    TT_B = np.linalg.inv(TB0) @ TP_0 @ TG_P2 @ np.linalg.inv(TGT)

                    q = quaternion_from_matrix(TT_B) # xyz
                    pose_goal = geometry_msgs.msg.Pose()
                    pose_goal.orientation.x = q[0]
                    pose_goal.orientation.y = q[1]
                    pose_goal.orientation.z = q[2]
                    pose_goal.orientation.w = q[3]
                    pose_goal.position.x = TT_B[0, 3]
                    pose_goal.position.y = TT_B[1, 3]
                    pose_goal.position.z = TT_B[2, 3]

                    goal_poses.append(pose_goal)

                    # self.send_pose_to_robot(TT_B, wait=True, cartesian=True)
                    # print(len(goal_poses))
                    # self.send_pose_to_robot(TT_B, wait=True)
            
            plan, _ = self.group.compute_cartesian_path(goal_poses, 0.01, 0.0)
            self.group.execute(plan, wait=True)
            self.group.clear_pose_targets()
            
            rospy.sleep(2)
            self.push_pub.publish(self.publish_msg)
            rospy.set_param('/load_model', True)

    def move_model_joint(self, model_name, joint_name, joint_value_rad):
        rospy.wait_for_service('/gazebo/set_model_configuration')
        success = False
        try:
            set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

            req = SetModelConfigurationRequest()
            req.model_name = model_name
            req.urdf_param_name = 'robot_description'  # Assuming your URDF is loaded with the parameter name 'robot_description'

            req.joint_names = [joint_name]
            req.joint_positions = [joint_value_rad]

            success = set_model_configuration(req)
            rospy.loginfo(f'Joint {joint_name} set to {joint_value_rad} for model {model_name}')
            return success.success
        except rospy.ServiceException as e:
            rospy.logerr(f'Failed to set joint configuration: {e}')
            return False


    def send_pose_to_robot(self, T, wait=True, cartesian=False):
        q = quaternion_from_matrix(T) # xyzw

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        pose_goal.position.x = T[0, 3]
        pose_goal.position.y = T[1, 3]
        pose_goal.position.z = T[2, 3]


        if cartesian:
            plan, _ = self.group.compute_cartesian_path([pose_goal], 0.01, 0.0)
            self.group.execute(plan, wait=wait)
            return

        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=wait)
        self.group.stop()
        self.group.clear_pose_targets()


if __name__ == '__main__':
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('push_operation_node', anonymous=True)
    # push_sub = rospy.Subscriber('/push_operation', Bool, callback)
    # rospack = rospkg.RosPack()
    # pkg_path = rospack.get_path('ao_manipulation')
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = 'arm'
    # group = moveit_commander.MoveGroupCommander(group_name)
    # ao = read_ao_dict_from_json(os.path.join(pkg_path, 'config', 'ao.txt'))
    # TA0 = np.load(os.path.join(pkg_path, 'config', 'TA0.npy'))
    # w_door = ao['s'][0]
    # h_door = ao['s'][1]
    # d_door = 0.018

    po = PushOperator()