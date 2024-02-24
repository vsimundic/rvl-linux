import rospy
import roslib
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import numpy as np
import os
from core.paths_packages import get_package_path_from_name
from trac_ik_python.trac_ik import IK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
roslib.load_manifest('robotiq_3f_gripper_control')
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput, Robotiq3FGripperRobotInput
from actionlib import SimpleActionClient
from core.transforms import get_frame_transform, matrix_to_pose
import socket
import time


class UR5Commander():
    def __init__(self):
        self.__robot = moveit_commander.RobotCommander()
        self.__group_name = 'arm'
        self.__group = moveit_commander.MoveGroupCommander(self.__group_name)
        self.__scene = moveit_commander.PlanningSceneInterface()
        self.ik = IK('base_link', 'tool0', timeout=0.05, solve_type='Distance')
        self.gripper_3f = Robotiq3FGripperRobotOutput()
        self.__gripper_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)   
        self.robot_ip = '192.168.10.14'
        self.robot_port = 30002

        self.T_B_S = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_B_S.npy')))
        self.T_G_T = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_G_T.npy')))
        self.T_G_T[2, 3] = 0.115
        # self.T_G_T[2, 3] -= 0.005
        self.T_C_T = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_C_T.npy')))
        
        self.joint_values_init = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'joint_values_init.npy')))
        self.T_T_B_home = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_T_B_home.npy')))

        self.__follow_joint_trajectory_client = SimpleActionClient('/trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        self.URScript_save_path = os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'script.txt')

        self.__create_init_scene()

        # test
        # T_3f_T = get_frame_transform('tool0', 'gripper_link')

        # T_3f_G = np.linalg.inv(self.T_G_T) @ T_3f_T
        # if T_3f_G is not None:
        #     np.save(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_3f_G.npy'), T_3f_G)



        


    def __create_init_scene(self):

        # Remove everything from the scene
        self.__scene.clear()

        # Add ground to scene
        is_on_scene = 'ground_box' in self.__scene.get_known_object_names()

        if not is_on_scene:
            box_pose = PoseStamped()
            box_pose.header.frame_id = 'base_link'
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.z = -(0.01 + self.T_B_S[2, 3])
            box_name = 'ground_box'
            self.__scene.add_box(box_name, box_pose, size=(1.2, 1.2, 0.01))


    def add_mesh_to_scene(self, mesh_filename: str, name: str, pose: np.ndarray):
        is_on_scene = name in self.__scene.get_known_object_names()

        if not is_on_scene:
            pose_ = self.__matrix_to_pose_stamped(pose, 'base_link')

            self.__scene.add_mesh(name=name, pose=pose_, filename=mesh_filename)


    def add_box_to_scene(self, object_name: str, pose: np.ndarray, frame_id: str, size: tuple):
        pose_s = self.__matrix_to_pose_stamped(T=pose, frame_id=frame_id)

        self.__scene.add_box(object_name, pose_s, size=size)


    def remove_from_scene(self, name: str):
        is_on_scene = name in self.__scene.get_known_object_names()

        if is_on_scene:
            self.__scene.remove_world_object(name=name)


    def send_pose_to_robot(self, T: np.ndarray, wait: bool=True, cartesian: bool=False):

        pose_goal = self.__matrix_to_pose(T)

        if cartesian:
            plan, _ = self.__group.compute_cartesian_path([pose_goal], 0.01, 0.0)
            self.__group.execute(plan, wait=wait)
            self.__group.clear_pose_targets()
            return

        self.__group.set_pose_target(pose_goal)
        plan = self.__group.go(wait=wait)
        self.__group.stop()
        self.__group.clear_pose_targets()


    def send_multiple_poses_to_robot(self, Ts: list, wait: bool=True, cartesian: bool=False, at_once: bool=True):
        
        poses_ = [self.__matrix_to_pose(T_) for T_ in Ts]

        if at_once:
            if cartesian:
                    plan, _ = self.__group.compute_cartesian_path(poses_, 0.01, 0.0)
                    self.__group.execute(plan, wait=wait)
            else:
                    self.__group.set_pose_targets(poses_)
                    plan = self.__group.plan()
                    self.__group.execute(plan[1], wait=wait)
                    # plan = self.__group.go(wait=wait)
        else:
            len_poses = len(poses_) - 1
            for idx, pose_goal in enumerate(poses_):
                rospy.loginfo('Pose %d/%d' % (idx, poses_))
                # pose_goal = self.__matrix_to_pose(T)
                if cartesian:
                    plan, _ = self.__group.compute_cartesian_path([pose_goal], 0.01, 0.0)
                    self.__group.execute(plan, wait=wait)
                else:
                    self.__group.set_pose_target(pose_goal)
                    plan = self.__group.go(wait=wait)
                
                self.__group.stop()
                self.__group.clear_pose_targets()


    def send_named_pose(self, name: str='up', wait: bool=True, cartesian: bool=False):
        self.__group.set_named_target(name)
        plan = self.__group.go(wait=wait)
        self.__group.stop()
        self.__group.clear_pose_targets()


    def send_joint_values_to_robot(self, joint_values,  wait: bool=True):
        self.__group.set_joint_value_target(joint_values)
        self.__group.go(wait=wait)
        self.__group.stop()


    def send_multiple_joint_space_poses_to_robot(self, joint_values_list: list, wait: bool=True):
        # for joints in joint_values_list:
        #     self.__group.set_joint_value_target(joints)
        
        # traj = self.__group.plan()
        # self.__group.execute(traj[1], wait=wait)

        self.__follow_joint_trajectory_client.wait_for_server()

        time_increments = np.linspace(0, 25, len(joint_values_list) + 1)

        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.__robot.get_joint_names(self.__group_name)[:6]
        
        for i, joint_values in enumerate(joint_values_list):
            print(joint_values)
            point = JointTrajectoryPoint()
            point.positions = joint_values
            # point.velocities = [0.1]*6
            point.time_from_start = rospy.Duration(time_increments[i+1])
            joint_trajectory.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory

        self.__follow_joint_trajectory_client.send_goal(goal)
        self.__follow_joint_trajectory_client.wait_for_result()


    def get_tool_pose_from_gripper_pose(self, T_G_B: np.ndarray):
        return T_G_B @ np.linalg.inv(self.T_G_T)


    def get_current_tool_pose(self):
        pose_ = self.__group.get_current_pose()

        return self.__pose_stamped_to_matrix(pose_)


    def save_current_tool_pose(self, file_path):
        T = self.get_current_tool_pose()
        np.save(file_path, T)


    def get_current_joint_values(self):
        return self.__group.get_current_joint_values()


    def save_current_joint_values(self, file_path):
        joint_values = self.get_current_joint_values()
        np.save(file_path, np.array(joint_values))


    def generate_URScript(self, q_array:np.ndarray, with_force_mode:bool):
        n = q_array.shape[0]
        txt = ''
        txt += 'def func():\n'
        if with_force_mode:
            txt += 'force_mode(tool_pose(), [0, 0, 1, 0, 0, 0], [0.0, 0.0, 25.0, 0.0, 0.0, 0.0], 2, [0.2, 0.2, 0.1, 0.60, 0.60, 0.35])\n'
        for i in range(n):
            txt += 'movej(' + str(q_array[i].tolist()) + ', a=1.4, v=0.2)\n'
        
        # if with_force_mode:
        #     txt += 'end_force_mode()\n'
        
        txt += 'end\n'
        txt += 'func()\n'

        with open(self.URScript_save_path, 'w') as f:
            f.write(txt)
            f.close()

    # def generate_URScript_poses(self, T_array:np.ndarray):
    #     n = q_array.shape[0]
    #     txt = ''
    #     txt += 'def func():\n'
    #     for i in range(n):
    #         txt += 'movej(' + str(q_array[i].tolist()) + ', a=1.4, v=0.2)\n'
    #     txt += 'end\n'
    #     txt += 'func()\n'
    #     with open(self.URScript_save_path, 'w') as f:
    #         f.write(txt)
    #         f.close()


    def send_URScript(self):

        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect((self.robot_ip, self.robot_port))
        f = open(self.URScript_save_path, 'rb')
        l = f.read()
        s.sendall(l)
    
        print('URScript sent!')
        data = s.recv(1024)
        s.close()

    def zero_ft_sensor(self):

        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect((self.robot_ip, self.robot_port))

        time.sleep(1)
        s.sendall(("zero_ftsensor()" + "\n").encode('utf-8')) 
        time.sleep(1)
        data=s.recv(1024)
        s.close()

    def open_gripper_pinch(self):

        self.gripper_3f.rACT = 1
        self.gripper_3f.rGTO = 1
        self.gripper_3f.rSPA = 255
        self.gripper_3f.rFRA = 150
        self.gripper_3f.rATR = 0
        self.gripper_3f.rMOD = 1
        self.gripper_3f.rPRA = 0
        
        # 3 second delay to let the gripper open
        start_time = time.time()
        
        # TODO: wait with Robotiq3FGripperInput message
        while True:
            self.__gripper_pub.publish(self.gripper_3f)
            rospy.sleep(0.1)
            end_time = time.time()
            if float(end_time - start_time) >= 3.0: # 3s
                break

    def close_gripper_pinch(self):

        self.gripper_3f.rACT = 1
        self.gripper_3f.rGTO = 1
        self.gripper_3f.rSPA = 255
        self.gripper_3f.rFRA = 150
        self.gripper_3f.rATR = 0
        self.gripper_3f.rMOD = 1
        self.gripper_3f.rPRA = 255
        
        # 3 second delay to let the gripper open
        start_time = time.time()
        
        # TODO: wait with Robotiq3FGripperInput message
        while True:
            self.__gripper_pub.publish(self.gripper_3f)
            rospy.sleep(0.1)
            end_time = time.time()
            if float(end_time - start_time) >= 3.0: # 3s
                break


    def get_inverse_kin(self, q_init, T):

        pose_ = matrix_to_pose(T)
        
        q = self.ik.get_ik(q_init, pose_.position.x, 
                       pose_.position.y, 
                       pose_.position.z,
                       pose_.orientation.x, 
                       pose_.orientation.y, 
                       pose_.orientation.z, 
                       pose_.orientation.w)
        if q is None:
            return None
        return list(q)

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
    
    
    @staticmethod
    def __matrix_to_pose_stamped(T: np.ndarray, frame_id: str):
        q = quaternion_from_matrix(T) # xyzw

        pose_s = PoseStamped()
        pose_ = Pose()
        pose_s.header.frame_id = frame_id
        pose_s.header.stamp = rospy.Time.now()

        pose_.orientation.x = q[0]
        pose_.orientation.y = q[1]
        pose_.orientation.z = q[2]
        pose_.orientation.w = q[3]
        pose_.position.x = T[0, 3]
        pose_.position.y = T[1, 3]
        pose_.position.z = T[2, 3]
        pose_s.pose = pose_
        return pose_s
    
    @staticmethod
    def __pose_stamped_to_matrix(pose: PoseStamped):
        pose_ = pose.pose
        q_ = [pose_.orientation.x, pose_.orientation.y, pose_.orientation.z, pose_.orientation.w]
        t_ = np.array([pose_.position.x, pose_.position.y, pose_.position.z])
        T = quaternion_matrix(q_)
        T[:3, 3] = np.array(t_)
        return T