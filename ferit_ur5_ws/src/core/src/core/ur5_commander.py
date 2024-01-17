import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import numpy as np
import os
from core.paths_packages import get_package_path_from_name

class UR5Commander():
    def __init__(self):
        self.__robot = moveit_commander.RobotCommander()
        self.__group_name = 'arm'
        self.__group = moveit_commander.MoveGroupCommander(self.__group_name)
        self.__scene = moveit_commander.PlanningSceneInterface()

        self.T_B_0 = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TB0.npy')))
        self.T_G0_T = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'TG0T.npy')))

        self.__create_init_scene()


    def __create_init_scene(self):

        # Remove everything from the scene
        self.__scene.clear()

        # Add ground to scene
        is_on_scene = 'ground_box' in self.__scene.get_known_object_names()

        if not is_on_scene:
            box_pose = PoseStamped()
            box_pose.header.frame_id = "base_link"
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.z = -(0.01 + self.T_B_0[2, 3])
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
                    self.__group.set_pose_target(poses_)
                    plan = self.__group.go(wait=wait)
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


    def get_tool_pose(self, T_G0_0:np.ndarray):
        return np.linalg.inv(self.T_B_0) @ T_G0_0 @ np.linalg.inv(self.T_G0_T)


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