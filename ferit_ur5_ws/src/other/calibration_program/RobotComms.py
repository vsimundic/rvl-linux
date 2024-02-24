import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class RobotComms:
    def __init__(self, move_group_name:str):
        try:
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface
            # self.group_name = "manipulator"
            self.group_name = move_group_name
            self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        except:
            print("Failed to initialize moveit!")
        
        
    def moveTo(self, px, py, pz, ox, oy, oz, ow):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = px
        pose_goal.position.y = py
        pose_goal.position.z = pz
        pose_goal.orientation.x = ox
        pose_goal.orientation.y = oy
        pose_goal.orientation.z = oz
        pose_goal.orientation.w = ow

        self.move_group.set_pose_target(pose_goal)

        plan=self.move_group.go(wait=True)
        self.move_group.stop()
        #self.move_group.clear_pose_tragets()
        return
        
    def pose(self):
        current_pose = self.move_group.get_current_pose().pose
        return current_pose
