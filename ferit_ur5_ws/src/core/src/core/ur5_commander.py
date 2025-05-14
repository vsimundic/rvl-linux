import rospy
import roslib
import moveit_commander
from moveit_msgs.msg import RobotTrajectory, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_matrix, quaternion_matrix
import numpy as np
import os
from core.paths_packages import get_package_path_from_name
from trac_ik_python.trac_ik import IK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
roslib.load_manifest('robotiq_3f_gripper_control')
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput, Robotiq3FGripperRobotInput
from actionlib import SimpleActionClient
from core.transforms import get_frame_transform, matrix_to_pose
import socket
import time
from actionlib import GoalStatus
from sensor_msgs.msg import JointState
from typing import Union


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
        self.pc_ip = '192.168.10.125'
        self.pc_port = 30002

        self.T_B_S = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_B_S.npy')))
        self.T_G_T = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_G_T.npy')))
        self.T_G_T[2, 3] = 0.115
        # self.T_G_T[2, 3] -= 0.005
        self.T_C_T = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_C_T.npy')))
        
        self.joint_values_init = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'joint_values_init.npy')))
        self.T_T_B_home = np.array(np.load(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_T_B_home.npy')))

        self.__follow_joint_trajectory_client = SimpleActionClient('/trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        self.URScript_save_path = os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'script.script')

        self.__create_init_scene()

        # test
        # T_3f_T = get_frame_transform('tool0', 'gripper_link')

        # T_3f_G = np.linalg.inv(self.T_G_T) @ T_3f_T
        # if T_3f_G is not None:
        #     np.save(os.path.join(get_package_path_from_name('gazebo_push_open'), 'config', 'T_3f_G.npy'), T_3f_G)



    def clear_planning_scene(self):
        self.__scene.clear()


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


    def send_joint_values_to_robot(self, joint_values: list,  wait: bool=True):
        self.__group.stop()
        self.__group.set_joint_value_target(joint_values)
        success = self.__group.go(wait=wait)
        self.__group.stop()
        return success


    def send_multiple_joint_space_poses_to_robot(self, joint_values_list: list, execute_time: float, wait: bool=True):
        def trajectory_result_callback(status, result):
            print("Trajectory execution completed.")


        self.__follow_joint_trajectory_client.wait_for_server()

        time_increments = np.linspace(0, execute_time, len(joint_values_list) + 1)

        joint_trajectory = JointTrajectory()
        print(self.__robot.get_joint_names(group=self.__group_name))
        joint_trajectory.joint_names = self.get_joint_names()
        
        for i, joint_values in enumerate(joint_values_list):
            print(joint_values)
            point = JointTrajectoryPoint()
            point.positions = joint_values
            # point.velocities = [0.1]*6
            point.time_from_start = rospy.Duration(time_increments[i+1])
            joint_trajectory.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory

        # state = self.__follow_joint_trajectory_client.send_goal_and_wait(goal)
        # state = self.__follow_joint_trajectory_client.send_goal(goal)
        state = self.__follow_joint_trajectory_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(execute_time), preempt_timeout=rospy.Duration(5.0))
        print("Done waiting for trajectory execution.")

        print(state)
        if state == GoalStatus.SUCCEEDED or state == GoalStatus.PREEMPTED:
            return True
        return False


    def send_multiple_joint_space_poses_to_robot2(self, joint_values_list: list):

        # Robot parameters
        max_joint_velocity = 0.5  # Maximum velocity in rad/s per joint
        max_joint_acceleration = 0.5  # Maximum acceleration in rad/s^2 per joint

        self.__follow_joint_trajectory_client.wait_for_server(timeout=rospy.Duration(10))

        joint_trajectory = JointTrajectory()
        
        joint_trajectory.joint_names = self.get_joint_names()
        
        total_time = 0.0  # Total execution time for the trajectory
        for i in range(len(joint_values_list)):
            point = JointTrajectoryPoint()
            point.positions = joint_values_list[i]
            if i > 0:
                # Calculate time increment based on velocity and acceleration limits
                time_increment = self.calculate_time_between_points(
                    joint_values_list[i - 1], joint_values_list[i],
                    max_joint_velocity, max_joint_acceleration
                )
            else:
                time_increment = 0.0

            total_time += time_increment
            point.time_from_start = rospy.Duration(total_time)
            joint_trajectory.points.append(point)

        correction_point = JointTrajectoryPoint()
        correction_point.positions = joint_values_list[-1]  # Same as last target
        correction_point.time_from_start = rospy.Duration(total_time + 0.2)  # Small buffer
        joint_trajectory.points.append(correction_point)


        goal = FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory

        state = self.__follow_joint_trajectory_client.send_goal_and_wait(
            goal,
            execute_timeout=rospy.Duration(5*total_time),  # Buffer time for execution
            preempt_timeout=rospy.Duration(5*total_time)
        )
        rospy.sleep(1.0)
        print("Done waiting for trajectory execution.")

        joint_states = rospy.wait_for_message("/joint_states", JointState)
        current_positions = {name: pos for name, pos in zip(joint_states.name, joint_states.position)}
        max_error = 0.0
        print("Final Joint States Check:")
        for joint_name in joint_trajectory.joint_names:
            actual = current_positions.get(joint_name, 0)
            expected = joint_values_list[-1][joint_trajectory.joint_names.index(joint_name)]
            error = np.abs(actual - expected)
            max_error = max(max_error, error)
            print(f"{joint_name}: Actual = {actual}, Expected = {expected}, Error = {error}")

        ERROR_THRESHOLD = 0.001  # 10e-6 rad (very small)
        if state == GoalStatus.SUCCEEDED or max_error < ERROR_THRESHOLD:
            if state != GoalStatus.SUCCEEDED:
                rospy.logwarn(f"Trajectory reported as ABORTED, but final error is {max_error} rad. Forcing SUCCESS.")
            return True
        rospy.logwarn("Trajectory execution finished but reported as ABORTED.")
        return False

    # def send_multiple_joint_space_poses_to_robot3(self, joint_values_list: list) -> bool:
    #     """
    #     Plans and executes a trajectory through a series of joint-space waypoints while considering obstacles in the 
    #     MoveIt PlanningSceneInterface.

    #     This function iteratively plans a trajectory for each joint-space waypoint provided in the input list. 
    #     If all waypoints are successfully planned, it combines them into a single RobotTrajectory and executes it.
    #     If planning fails for any waypoint, the function stops further planning and logs a warning.

    #     Parameters:
    #     ----------
    #     joint_values_list : list
    #         A list of joint-space configurations, where each configuration is a list of joint values (floats) 
    #         corresponding to the robot's joint positions.

    #     Returns:
    #     -------
    #     bool
    #         True if a collision-free trajectory is successfully planned and executed through all waypoints. 
    #         False if planning fails for any of the waypoints.
    #     """
    #     trajectory = RobotTrajectory()
    #     success = True
    #     total_time = 0.0  # Keeps track of cumulative time for waypoints
    #     max_velocity = 1.0
    #     max_acceleration = 0.5

    #     # Set joint names
    #     trajectory.joint_trajectory.joint_names = self.__group.get_active_joints()
        
    #     # Create a RobotState object to manually update the start state
    #     current_state = RobotState()

    #     for index, joints in enumerate(joint_values_list):
    #         self.__group.set_joint_value_target(joints)

    #         # Update the start state
    #         if index == 0:
    #             # For the first waypoint, use the robot's current state
    #             self.__group.set_start_state_to_current_state()
    #         else:
    #             # For subsequent waypoints, use the last point of the previously planned trajectory
    #             last_trajectory_point = trajectory.joint_trajectory.points[-1]
    #             joint_state_names = self.__group.get_active_joints()
    #             joint_state_positions = last_trajectory_point.positions

    #             # Populate the RobotState object with the last trajectory point
    #             joint_state = JointState()
    #             joint_state.name = joint_state_names
    #             joint_state.position = joint_state_positions
    #             current_state.joint_state = joint_state
    #             self.__group.set_start_state(current_state)

    #         plan = self.__group.plan()
    #         if plan[0] and plan[1].joint_trajectory.points:
    #             for point_index, point in enumerate(plan[1].joint_trajectory.points):
    #                 # Dynamically calculate time between points
    #                 if index == 0 and point_index == 0:
    #                     continue  # Skip time calculation for the first point
    #                 prev_point = plan[1].joint_trajectory.points[point_index - 1] if point_index > 0 else trajectory.joint_trajectory.points[-1]
    #                 time_increment = self.calculate_time_between_points(
    #                     prev_point.positions, point.positions, max_velocity, max_acceleration)
    #                 total_time += time_increment
    #                 point.time_from_start = rospy.Duration.from_sec(total_time)

    #             # Append the planned trajectory points to the composite trajectory
    #             trajectory.joint_trajectory.points.extend(plan[1].joint_trajectory.points)
    #         else:
    #             rospy.logwarn('Planning failed for waypoints: %s' % joints)
    #             success = False
    #             break
        
    #         if not trajectory.joint_trajectory.joint_names:
    #             rospy.logerr("Trajectory specifies no joint names.")
    #             return False
    #         if not trajectory.joint_trajectory.points:
    #             rospy.logerr("Trajectory contains no points.")
    #             return False
        
    #     # Execute the trajectory if successful
    #     if success:
    #         success = self.__group.execute(trajectory, wait=True)
    #     else:
    #         rospy.logwarn("Failed to plan a collision-free trajectory through waypoints.")
        
    #     return success


    @staticmethod
    def calculate_time_between_points(start, end, max_velocity, max_acceleration):
        """
        Calculate time increment between two joint positions based on velocity and acceleration limits.
        """
        import numpy as np

        # Compute joint-wise differences
        position_differences = np.abs(np.array(end) - np.array(start))

        # Compute time required for each joint (velocity and acceleration limited)
        time_velocity = position_differences / max_velocity
        time_acceleration = np.sqrt(2 * position_differences / max_acceleration)

        # Return the maximum time required across all joints
        return max(max(time_velocity), max(time_acceleration))



    def get_tool_pose_from_gripper_pose(self, T_G_B: np.ndarray):
        return T_G_B @ np.linalg.inv(self.T_G_T)


    def get_current_tool_pose(self):
        pose_ = self.__group.get_current_pose()

        return self.__pose_stamped_to_matrix(pose_)


    def forward_kinematics(self, joint_values: list) -> np.ndarray:

        rospy.wait_for_service('/compute_fk')
        fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        fk_request = GetPositionFKRequest()
        fk_request.fk_link_names = ['tool0']  # or your end-effector link
        fk_request.header.frame_id = 'base_link'

        joint_state = JointState()
        joint_state.name = self.get_joint_names()
        joint_state.position = joint_values
        fk_request.robot_state = RobotState(joint_state=joint_state)

        try:
            response = fk_service(fk_request)
            if response.error_code.val == response.error_code.SUCCESS:
                pose_stamped = response.pose_stamped[0]
                return self.__pose_stamped_to_matrix(pose_stamped)
            else:
                rospy.logerr("FK computation failed with error code: %s", response.error_code.val)
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call to /compute_fk failed: %s", e)
            return None


    def save_current_tool_pose(self, file_path):
        T = self.get_current_tool_pose()
        np.save(file_path, T)


    def get_current_joint_values(self) -> list:
        return self.__group.get_current_joint_values()


    def save_current_joint_values(self, file_path):
        joint_values = self.get_current_joint_values()
        np.save(file_path, np.array(joint_values))


    # def generate_URScript(self, q_array:np.ndarray, with_force_mode:bool):
    #     n = q_array.shape[0]
    #     txt = ''
    #     txt += 'def func():\n'
    #     if with_force_mode:
    #         txt += 'force_mode(tool_pose(), [0, 0, 1, 0, 0, 0], [0.0, 0.0, 25.0, 0.0, 0.0, 0.0], 2, [0.2, 0.2, 0.1, 0.60, 0.60, 0.35])\n'
    #     for i in range(n):
    #         txt += 'movej(' + str(q_array[i].tolist()) + ', a=1.4, v=0.2)\n'
        
    #     # if with_force_mode:
    #     #     txt += 'end_force_mode()\n'
        
    #     # Needs debugging
    #     txt += f'socket_open(\"{self.pc_ip}\", {self.pc_port})\n'
    #     txt += f'socket_send_string(\"Script completed\")\n'
    #     txt += f'socket_close()\n'

    #     txt += f'end\n'
    #     txt += f'func()\n'

    #     with open(self.URScript_save_path, 'w') as f:
    #         f.write(txt)
    #         f.close()

    def generate_URScript(self, q_array: np.ndarray, with_force_mode: bool, script_path:str=None):
        """
        Generate a URScript that zeroes the force-torque sensor, enters force mode (optional),
        executes joint motions, and notifies the PC via socket.
        """
        n = q_array.shape[0]
        indent = '    '
        lines = []

        lines.append('def func():')

        if with_force_mode:
            lines.append(f'{indent}# Zero the force-torque sensor before force mode')
            lines.append(f'{indent}zero_ftsensor()')
            lines.append(f'{indent}sleep(0.1)  # Allow sensor to settle')

            lines.append(f'{indent}# Start force mode')
            lines.append(f'{indent}force_mode(tool_pose(), [0, 0, 0, 0, 0, 0], '
                        f'[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], '
                        f'2, [0.2, 0.2, 0.1, 0.60, 0.60, 0.35])')

        lines.append(f'{indent}# Execute joint motions')
        for i in range(n):
            q_list = q_array[i].tolist()
            lines.append(f'{indent}movej({q_list}, a=1.4, v=0.2)')

        if with_force_mode:
            lines.append(f'{indent}# Exit force mode')
            lines.append(f'{indent}end_force_mode()')

        lines.append(f'{indent}# Notify PC')
        lines.append(f'{indent}socket_open("{self.pc_ip}", {self.pc_port})')
        lines.append(f'{indent}socket_send_string("Script completed")')
        lines.append(f'{indent}socket_close()')

        lines.append('end')
        lines.append('func()')
        
        with open(self.URScript_save_path if script_path is None else script_path, 'w') as f:
            f.write('\n'.join(lines))

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


    def send_URScript(self, get_feedback: False, script_path:str=None):

        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect((self.robot_ip, self.robot_port))
        f = open(self.URScript_save_path if script_path is None else script_path, 'rb')
        l = f.read()
        s.sendall(l)
    
        print('URScript sent!')
        data = s.recv(1024)
        s.close()

        if get_feedback:
            server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            host = self.pc_ip
            port = self.pc_port
            server_socket.bind((host, port))
            server_socket.listen(1)
            
            print(f"Waiting for a connection on {host}:{port}...")

            connection, client_address = server_socket.accept()
            try:
                print(f"Connection from {client_address} established")
                
                # Receive the data in small chunks and retransmit it
                while True:
                    data = connection.recv(1024)
                    if data:
                        print("Received message:", data.decode('utf-8'))
                        break  # Exit the loop after receiving the message
                    else:
                        break  # No more data from the client
                    
            finally:
                # Clean up the connection
                connection.close()
                print("Connection closed")


    def zero_ft_sensor(self):

        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect((self.robot_ip, self.robot_port))

        time.sleep(1)
        s.sendall(("zero_ftsensor()" + "\n").encode('utf-8')) 
        time.sleep(1)
        data=s.recv(1024)
        s.close()
    

    def get_data_from_ft_sensor(self):

        sensor_port = 63351
        output_file_location = '/home/RVLuser/ferit_ur5_ws/ft_sensor_data.csv'
        write_object = True
        socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        publisher_object = publisher_object
        is_published = False

        try:
            print('Connecting to UR5 at ' + self.robot_ip)
            socket_object.connect((self.robot_ip, sensor_port))
  
            if self.write_object is True:
                print('File Write Location: ' + output_file_location)
                f = open(output_file_location, 'w')
                       
            try:
                print('Writing in %s, Press Ctrl + Z to stop' % output_file_location)
                
                while True:
                    data = self.socket_object.recv(1024) #procita bajte
                    
                    bytes_unpacked_data = data.decode('utf-8')[1:-1]
                    #print(bytes_unpacked_data)
                    dataF = bytes_unpacked_data.split(" , ")
                    print(str(dataF))
                    print("\n")

                    #force_torque_values = (self.socket_object.recv(1024).replace("(","*")).replace(")","*")
                    
                    #value_list = force_torque_values.split("*")
                   
                    
                    #rospy.loginfo(value_list [1])
                    ##rospy.loginfo(dataF)



                    #self.publisher_object.publish(value_list[1])
                    # self.publisher_object.publish(dataF)
                    ##self.publisher_rate.sleep()
                

                    #if self.write_object is True:       
                    #    f.write(strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime()) + ": " +force_torque_values)
                                              
            except KeyboardInterrupt:
                        
                f.close()
                socket_object.close()

                return False

        except Exception as e:

            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))

            return False


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


    def check_joint_feasibility(self, start_state_joints, joint_values):
        """
        Checks if a given joint configuration is collision-free.
        
        Args:
            joint_values: List of joint angles.

        Returns:
            True if collision-free, False otherwise.
        """
        start_state = moveit_commander.RobotState()
        start_state.joint_state.name = self.__group.get_active_joints()
        start_state.joint_state.position = start_state_joints
        self.__group.set_start_state(start_state)
        self.__group.set_joint_value_target(joint_values)

        # Perform collision checking
        plan = self.__group.plan()
        if plan and len(plan[1].joint_trajectory.points) > 0:
            return True  # Valid plan exists, meaning no collision
        return False  # Collision detected
    

    def is_state_valid(self, q: list) -> bool:
        """
        Validates a single joint configuration using the /check_state_validity service.

        Args:
            q (list): A single joint configuration [rad].

        Returns:
            bool: True if the configuration is collision-free, False otherwise.
        """
        from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

        joint_names = self.get_joint_names()

        if len(q) != 6:
            rospy.logerr("Expected a joint state with 6 values")
            return False

        rospy.wait_for_service('/check_state_validity')
        check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        req = GetStateValidityRequest()
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = joint_names
        req.robot_state.joint_state.position = q
        req.group_name = self.__group_name

        try:
            res = check_state_validity(req)
            return res.valid
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False


    def validate_trajectory_points(self, q_traj: np.ndarray) -> bool:
        """
        Validates each joint state in the trajectory using the /check_state_validity service.

        Args:
            q_traj (np.ndarray): Joint trajectory of shape (N, 6)

        Returns:
            bool: True if all joint states are collision-free, False otherwise.
        """
        from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

        joint_names = self.get_joint_names()

        if q_traj.ndim != 2 or q_traj.shape[1] != 6:
            rospy.logerr("Expected joint trajectory of shape (N, 6)")
            return False

        rospy.wait_for_service('/check_state_validity')
        check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        for i, q in enumerate(q_traj):
            req = GetStateValidityRequest()
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = joint_names
            req.robot_state.joint_state.position = q.tolist()
            req.group_name = self.__group_name

            try:
                res = check_state_validity(req)
                if not res.valid:
                    rospy.logwarn(f"State {i} in trajectory is in collision.")
                    return False
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return False

        rospy.loginfo("All states in trajectory are collision-free.")
        return True

    def get_inverse_kin(self, q_init: list, T: np.ndarray) -> Union[list, None]:

        pose_ = matrix_to_pose(T)
        
        q = self.ik.get_ik(q_init, pose_.position.x, 
                       pose_.position.y, 
                       pose_.position.z,
                       pose_.orientation.x, 
                       pose_.orientation.y, 
                       pose_.orientation.z, 
                       pose_.orientation.w)
        if q is None:
            return None  # No valid IK solution found

        q = list(q)  # Convert to list for MoveIt!

        # Check if the solution is collision-free
        if self.check_joint_feasibility(q_init, q):
            return q  # Return the collision-free solution
        else:
            rospy.logwarn("IK solution is in collision!")
            return None

    def get_inverse_kin_builtin(self, q_init: list, T: np.ndarray) -> Union[list, None]:
        """
        Uses built-in set_pose_targets() and plan() to get inverse kin.
        """
        pose_ = matrix_to_pose(T)

        start_state = moveit_commander.RobotState()
        start_state.joint_state.name = self.__group.get_active_joints()
        start_state.joint_state.position = q_init
        self.__group.set_start_state(start_state)
        # self.__group.set_start_state_to_current_state()
        
        self.__group.set_pose_target(pose_)
        plan = self.__group.plan()  # Directly get the plan (not a tuple)
        
        if plan[0] and len(plan[1].joint_trajectory.points) > 0:
            traj = []
            for pt in plan[1].joint_trajectory.points:
                traj.append(list(pt.positions))
            
            return traj[-1], traj
        else:
            return None, None

    def get_joint_names(self):
        return self.__robot.get_joint_names(self.__group_name)[1:7]


    def reset_moveit_joints(self, default_positions=None):
        """
        Resets the robot joints to default positions.
        
        Args:
            joint_names (list): List of joint names.
            default_positions (list): List of default joint values (optional).
        """

        joint_names = self.get_joint_names()

        if default_positions is None:
            default_positions = [0.0] * len(joint_names)  # Default to zero positions

        rospy.loginfo("Resetting robot joints...")

        # Publish new joint state
        pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        joint_msg = JointState()
        joint_msg.name = joint_names
        joint_msg.position = default_positions
        joint_msg.header.stamp = rospy.Time.now()

        for _ in range(5):  # Publish multiple times to ensure reception
            pub.publish(joint_msg)
            rospy.sleep(0.1)

        rospy.loginfo(f"Robot joints reset to: {default_positions}")
        
        # Ensure MoveIt! updates its internal state
        self.__group.set_joint_value_target(default_positions)
        self.__group.go(wait=True)
        
        # # Reset controllers if needed
        # try:
        #     rospy.wait_for_service('/move_group/stop', timeout=5)
        #     move_group_stop = rospy.ServiceProxy('/move_group/stop', Empty)
        #     move_group_stop()
        #     rospy.loginfo("MoveIt! execution stopped.")
        # except rospy.ServiceException as e:
        #     rospy.logwarn(f"Failed to stop MoveIt!: {e}")

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
    
    @staticmethod
    def cancel_active_moveit_goal():
        """
        Cancels the active MoveIt! goal via action client.
        """
        import actionlib
        from moveit_msgs.msg import MoveGroupAction
        rospy.loginfo("Connecting to MoveIt! action client...")
        client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        client.wait_for_server()
        
        rospy.loginfo("Canceling the active goal...")
        client.cancel_all_goals()
        rospy.loginfo("MoveIt! goal canceled.")