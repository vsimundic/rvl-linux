import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import numpy as np

class UR5Commander():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = 'arm'
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

    def __matrix_to_pose(self, T: np.ndarray):
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

    def send_pose_to_robot(self, T: np.ndarray, wait: bool=True, cartesian: bool=False):

        pose_goal = self.__matrix_to_pose(T)

        if cartesian:
            plan, _ = self.group.compute_cartesian_path([pose_goal], 0.01, 0.0)
            self.group.execute(plan, wait=wait)
            self.group.clear_pose_targets()
            return

        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=wait)
        self.group.stop()
        self.group.clear_pose_targets()

    def send_multiple_poses_to_robot(self, Ts: list, wait: bool=True, cartesian: bool=False):
        for T in Ts:
            pose_goal = self.__matrix_to_pose(T)
            if cartesian:
                plan, _ = self.group.compute_cartesian_path([pose_goal], 0.01, 0.0)
                self.group.execute(plan, wait=wait)
                # return
            else:
                self.group.set_pose_target(pose_goal)
                plan = self.group.go(wait=wait)
            
            self.group.stop()
            self.group.clear_pose_targets()