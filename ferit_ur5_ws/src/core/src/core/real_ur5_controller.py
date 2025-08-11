import rospy
import numpy as np
import moveit_commander
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory, RobotState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from moveit_msgs.msg import RobotState as RobotStateMsg
from xela_server_ros.msg import SensStream
from std_srvs.srv import Trigger
from core.transforms import pose_to_matrix, matrix_to_pose, matrix_to_pose_stamped
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
import RVLPYDDManipulator as rvlpy_dd_man
from core.transforms import pose_stamped_to_matrix
from typing import Union
import open3d as o3d
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from fast_collision_checker import FastCollisionChecker

class UR5Controller:
    def __init__(self, move_group="arm", 
                 action_topic="/scaled_pos_joint_traj_controller/follow_joint_trajectory",
                 rvl_cfg_path="/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg"):
        moveit_commander.roscpp_initialize([])
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = move_group
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        
        self.group.set_planner_id("RRTstar")

        # Get reference to planning scene monitor
        self.collision_checker = FastCollisionChecker(self.group_name)
        
        # Camera pose
        self.T_C_6 = np.eye(4)

        # Gripper pose
        self.T_G_6 = np.eye(4)
        self.gripper_mesh: o3d.geometry.TriangleMesh = None

        # Pose in world
        self.T_0_W = np.eye(4)

        # RVL IK solver
        self.rvl_ik_solver = rvlpy_dd_man.PYDDManipulator()
        self.rvl_ik_solver.create(rvl_cfg_path)
        # self.rvl_ik_solver.set_robot_pose(T_R_W)

        self.T_G_6 = self.rvl_ik_solver.get_T_G_6()

        self.joint_names = self.group.get_active_joints()
        self.joint_states = None
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        self.client = actionlib.SimpleActionClient(action_topic, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for trajectory action server...")
        # self.client.wait_for_server()
        rospy.loginfo("Connected to FollowJointTrajectory action server.")

        self.active_goal = None
        self.latest_wrench = None
        self.force_threshold = 30.0  # Newtons

        self.tactile_contact_established = False
        self.tactile_data = None
        self.tactile_wrench = None

        # Use real FT sensor topic
        rospy.Subscriber("/robotiq_ft_sensor/wrench", WrenchStamped, self.wrench_callback)

        self.tactile_sub = rospy.Subscriber("/xServTopic", SensStream, self.tactile_data_callback)

        self.add_ground_plane()

        # rospy.wait_for_service('/check_state_validity')
        # self.check_state_validity = rospy.ServiceProxy('/check_state_validity', GetStateValidity)


    def add_ground_plane(self, z_height=0.0, size=(2.0, 2.0), timeout=2.0):
        """
        Adds a ground plane box to the planning scene at the specified height.
        """
        rospy.loginfo("Adding ground plane to planning scene...")

        ground_pose = PoseStamped()
        ground_pose.header.frame_id = self.robot.get_planning_frame()  # usually "base_link"
        ground_pose.pose.orientation.w = 1.0
        ground_pose.pose.position.x = 0.0
        ground_pose.pose.position.y = 0.0
        ground_pose.pose.position.z = z_height  # slightly below to act as "floor"

        self.scene.add_box("ground_plane", ground_pose, size=(size[0], size[1], 0.02))

        # Wait for scene update
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start_time < timeout):
            if "ground_plane" in self.scene.get_known_object_names():
                rospy.loginfo("Ground plane added.")
                return True
            rospy.sleep(0.1)

        rospy.logwarn("Timed out waiting for ground plane to be added.")
        return False

    def add_mesh_to_scene(self, mesh_path, mesh_name, mesh_pose, timeout=2.0, verbose=False):
        """
        Adds a mesh to the planning scene at the specified pose.
        """
        if verbose:
            rospy.loginfo("Adding mesh to planning scene...")

        mesh_pose_stamped = matrix_to_pose_stamped(mesh_pose, 'base_link')
        # mesh_pose_stamped.header.frame_id = self.robot.get_planning_frame()

        self.scene.add_mesh(mesh_name, mesh_pose_stamped, mesh_path)

        # Wait for scene update
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start_time < timeout):
            if mesh_name in self.scene.get_known_object_names():
                if verbose:
                    rospy.loginfo("Mesh added.")
                return True
            rospy.sleep(0.1)

        rospy.logwarn("Timed out waiting for mesh to be added.")
        return False

    def add_box_to_scene(self, box_name, box_pose, size=(0.1, 0.1, 0.1), timeout=2.0):
        """
        Adds a box to the planning scene at the specified pose.
        """
        rospy.loginfo("Adding box to planning scene...")

        box_pose_stamped = matrix_to_pose_stamped(box_pose, 'base_link')
        # box_pose_stamped.header.frame_id = self.robot.get_planning_frame()

        self.scene.add_box(box_name, box_pose_stamped, size=size)

        # Wait for scene update
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and (rospy.get_time() - start_time < timeout):
            if box_name in self.scene.get_known_object_names():
                rospy.loginfo("Box added.")
                return True
            rospy.sleep(0.1)

        rospy.logwarn("Timed out waiting for box to be added.")
        return False

    def remove_box_from_scene(self, box_name):
        """
        Removes a box from the planning scene.
        """
        rospy.loginfo("Removing box from planning scene...")

        self.scene.remove_world_object(box_name)

        # Wait for scene update
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and box_name in self.scene.get_known_object_names():
            rospy.sleep(0.1)

        rospy.loginfo("Box removed.")
        return True

    def remove_mesh_from_scene(self, mesh_name):
        """
        Removes a mesh from the planning scene.
        """
        rospy.loginfo("Removing mesh from planning scene...")

        self.scene.remove_world_object(mesh_name)

        # Wait for scene update
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and mesh_name in self.scene.get_known_object_names():
            rospy.sleep(0.1)

        rospy.loginfo("Mesh removed.")
        return True

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def wrench_callback(self, msg):
        self.latest_wrench = msg.wrench

    def tactile_data_callback(self, msg):
        # Convert SensStream to numpy array
        if len(msg.sensors) < 1:
            rospy.logwarn("No tactile sensors found in the message.")
            self.tactile_sub.unregister()
            return
        self.tactile_data = np.zeros(shape=(len(msg.sensors), len(msg.sensors[0].forces), 3))
        for i, sensor in enumerate(msg.sensors):
            for j, force in enumerate(sensor.forces):
                self.tactile_data[i, j, 0] = force.x
                self.tactile_data[i, j, 1] = force.y
                self.tactile_data[i, j, 2] = force.z

    def get_current_joint_values(self):
        return np.array(self.group.get_current_joint_values())

    def get_current_tool_pose(self):
        pose_msg = self.group.get_current_pose().pose
        return pose_to_matrix(pose_msg)

    def move_to_joint_goal(self, joint_goal: np.ndarray):
        assert joint_goal.shape == (6,)
        success = self.group.go(joint_goal.tolist(), wait=True)
        self.group.stop()
        return success

    def move_to_pose_matrix(self, pose_matrix: np.ndarray):
        assert pose_matrix.shape == (4, 4)
        pose_msg = matrix_to_pose(pose_matrix)
        self.group.set_pose_target(pose_msg)
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def plan_to_joint_goal(self, joint_goal: np.ndarray):
        """
        Plans a trajectory to a joint goal using MoveIt.

        Args:
            joint_goal (np.ndarray): 6-element array of joint positions [rad]

        Returns:
            np.ndarray: Planned trajectory as an (N, 6) NumPy array of joint positions
            bool: True if planning succeeded, False otherwise
        """
        assert joint_goal.shape == (6,)

        self.group.set_joint_value_target(joint_goal.tolist())
        success, plan, _, _ = self.group.plan()
        if not success:
            rospy.logwarn("MoveIt failed to plan to joint goal.")
            return None, False
        
        try:
            traj_msg = plan.joint_trajectory
            joint_traj_np = np.array([pt.positions for pt in traj_msg.points])

            rospy.loginfo("Planning successful. Trajectory has %d points.", len(joint_traj_np))
            return joint_traj_np, True
        except Exception as e:
            rospy.logerr("Error extracting planned trajectory: %s", str(e))
            return None, False

    def plan_to_joint_goal2(self, joint_goal: np.ndarray):
        """
        Plans a trajectory to a joint goal using MoveIt.

        Args:
            joint_goal (np.ndarray): 6-element array of joint positions [rad]

        Returns:
            np.ndarray: Planned trajectory as an (N, 6) NumPy array of joint positions
            bool: True if planning succeeded, False otherwise
        """
        assert joint_goal.shape == (6,)

        self.group.set_joint_value_target(joint_goal.tolist())
        success, plan, _, _ = self.group.plan()
        if not success:
            rospy.logwarn("MoveIt failed to plan to joint goal.")
            return None, False
        return plan.joint_trajectory, True


    def plan_to_joint_goals(self, joint_goals: np.ndarray):
        """
        Plans sequential trajectories through multiple joint goals using MoveIt.

        Args:
            joint_goals (np.ndarray): (N, 6) array of joint configurations [rad].

        Returns:
            np.ndarray: Planned trajectory as an (M, 6) array (may contain more than N rows due to interpolation).
            bool: True if all plans succeeded, False otherwise.
        """
        assert joint_goals.ndim == 2 and joint_goals.shape[1] == 6
        full_trajectory = []
        
        for i in range(len(joint_goals) - 1):
            start = joint_goals[i]
            goal = joint_goals[i + 1]

            # Set start and goal
            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(goal.tolist())

            # Manually set the start state to match `start`
            start_state = moveit_commander.RobotState()
            start_state.joint_state.name = self.group.get_active_joints()
            start_state.joint_state.position = start.tolist()
            self.group.set_start_state(start_state)

            success, plan, _, _ = self.group.plan()
            if not success or not plan.joint_trajectory.points:
                rospy.logwarn(f"Planning failed between waypoint {i} and {i+1}.")
                return None, False

            segment = [pt.positions for pt in plan.joint_trajectory.points]
            # Avoid repeating the last point of the previous segment
            if i > 0:
                segment = segment[1:]
            full_trajectory.extend(segment)

        trajectory_np = np.array(full_trajectory)
        rospy.loginfo("Successfully planned through %d waypoints. Total trajectory points: %d.",
                    len(joint_goals), len(trajectory_np))
        return trajectory_np, True

    def plan_to_joint_goals2(self, joint_goals: np.ndarray):
        """
        Plans sequential trajectories through multiple joint goals using MoveIt.

        Args:
            joint_goals (np.ndarray): (N, 6) array of joint configurations [rad].

        Returns:
            JointTrajectory: Combined planned trajectory message.
            bool: True if all plans succeeded, False otherwise.
        """
        assert joint_goals.ndim == 2 and joint_goals.shape[1] == 6

        combined_traj = JointTrajectory()
        combined_traj.joint_names = self.group.get_active_joints()
        time_offset = 0.0  # To accumulate time_from_start between segments

        for i in range(len(joint_goals) - 1):
            start = joint_goals[i]
            goal = joint_goals[i + 1]

            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(goal.tolist())

            # Manually override the start state
            start_state = moveit_commander.RobotState()
            start_state.joint_state.name = combined_traj.joint_names
            start_state.joint_state.position = start.tolist()
            self.group.set_start_state(start_state)

            success, plan, _, _ = self.group.plan()
            if not success or not plan.joint_trajectory.points:
                rospy.logwarn(f"Planning failed between waypoint {i} and {i+1}.")
                return None, False

            # Append points, adjusting time_from_start
            for j, pt in enumerate(plan.joint_trajectory.points):
                if i > 0 and j == 0:
                    continue  # Skip duplicate start
                new_pt = JointTrajectoryPoint()
                new_pt.positions = pt.positions
                new_pt.velocities = pt.velocities
                new_pt.accelerations = pt.accelerations
                new_pt.effort = pt.effort
                new_pt.time_from_start = pt.time_from_start + rospy.Duration(time_offset)
                combined_traj.points.append(new_pt)

            # Update time offset for next segment
            if combined_traj.points:
                time_offset = combined_traj.points[-1].time_from_start.to_sec()

        rospy.loginfo("Successfully planned through %d waypoints. Total trajectory points: %d.",
                    len(joint_goals), len(combined_traj.points))
        return combined_traj, True


    def send_joint_trajectory_action(self, joint_points: np.ndarray, max_velocity=1.0, max_acceleration=1.0) -> bool:
        assert joint_points.shape[1] == 6

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        cumulative_time = 0.0

        for i, point in enumerate(joint_points):
            pt = JointTrajectoryPoint()
            pt.positions = point.tolist()
            dt = self.calculate_time_between_points(joint_points[i - 1], point, max_velocity, max_acceleration) if i > 0 else 0.0
            cumulative_time += dt
            pt.time_from_start = rospy.Duration(cumulative_time)
            traj.points.append(pt)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        self.force_violation = False

        try:
            rospy.loginfo("Sending trajectory goal...")
            self.client.send_goal(goal)
            self.active_goal = self.client

            self.client.wait_for_result()

            result = self.client.get_result()
            state = self.client.get_state()

            if self.force_violation:
                rospy.logwarn("Trajectory canceled due to force threshold.")
                return False

            if state == 3:  # SUCCEEDED
                rospy.loginfo("Trajectory executed successfully.")
                return True
            else:
                rospy.logwarn("Trajectory failed or was interrupted. State: %s", str(state))
                return False
        except Exception as e:
            rospy.logerr("Trajectory execution error: %s", str(e))
            return False

    def send_joint_trajectory_action2(self, trajectory: Union[np.ndarray, JointTrajectory], max_velocity=1.0, max_acceleration=1.0) -> bool:
        if isinstance(trajectory, np.ndarray):
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            cumulative_time = 0.0
            for i, point in enumerate(trajectory):
                pt = JointTrajectoryPoint()
                pt.positions = point.tolist()
                dt = self.calculate_time_between_points(trajectory[i - 1], point, max_velocity, max_acceleration) if i > 0 else 0.0
                cumulative_time += dt
                pt.time_from_start = rospy.Duration(cumulative_time)
                traj.points.append(pt)
            trajectory_ = traj
        elif isinstance(trajectory, JointTrajectory):
            trajectory_ = trajectory
        else: 
            rospy.logerr("Invalid trajectory type. Expected np.ndarray or JointTrajectory.")
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory_
        self.force_violation = False

        try:
            rospy.loginfo("Sending trajectory goal...")
            self.client.send_goal(goal)
            self.active_goal = self.client

            self.client.wait_for_result()

            result = self.client.get_result()
            state = self.client.get_state()

            if self.force_violation:
                rospy.logwarn("Trajectory canceled due to force threshold.")
                return False

            if state == 3:  # SUCCEEDED
                rospy.loginfo("Trajectory executed successfully.")
                return True
            else:
                rospy.logwarn("Trajectory failed or was interrupted. State: %s", str(state))
                return False
        except Exception as e:
            rospy.logerr("Trajectory execution error: %s", str(e))
            return False


    def cancel_trajectory(self):
        if self.active_goal:
            rospy.logwarn("Cancelling active trajectory...")
            self.active_goal.cancel_goal()

    def zero_ft_sensor(self) -> bool:
        """
        Call the external FT sensor zeroing service.
        Returns True if successful, False otherwise.
        """
        service_name = "/robotiq_ft_streamer/zero"
        rospy.loginfo("[UR5Controller] Calling FT sensor zeroing service...")

        try:
            rospy.wait_for_service(service_name, timeout=3.0)
            zero_service = rospy.ServiceProxy(service_name, Trigger)
            response = zero_service()
            if response.success:
                rospy.loginfo("[UR5Controller] FT sensor zeroed: %s", response.message)
                return True
            else:
                rospy.logwarn("[UR5Controller] FT zeroing failed: %s", response.message)
                return False
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("[UR5Controller] Failed to call FT zeroing service: %s", str(e))
            return False

    def get_current_wrench(self) -> np.ndarray:
        """
        Returns the latest force-torque values as a 6-element numpy array:
        [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        if self.latest_wrench is None:
            rospy.logwarn_throttle(5, "[UR5Controller] No FT data received yet.")
            return np.zeros(6)

        f = self.latest_wrench.force
        t = self.latest_wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # def get_current_tactile_data(self) -> np.ndarray:
    #     """
    #     Returns the latest tactile data as a n-element numpy array.
    #     The size of the array depends on the number of tactile sensors and taxels.
    #     """
    #     if self.tactile_data is None:

    #     if self.latest_wrench is None:
    #         rospy.logwarn_throttle(5, "[UR5Controller] No FT data received yet.")
    #         return np.zeros(6)

    #     f = self.latest_wrench.force
    #     t = self.latest_wrench.torque
    #     return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def get_all_ik_solutions_rvl(self, pose_matrix: np.ndarray, check_env_collisions: bool = False, check_self_collisions: bool = False):
        """
        Compute all inverse kinematics solutions using RVL IK solver. This function also checks for collisions
        in the environment and self-collisions if specified and normalizes the joint solutions.
        
        Args:
            pose_matrix (np.ndarray): 4x4 transformation matrix representing desired end-effector pose (T_G_0)

        Returns:
            solutions (np.ndarray): Nx6 array of joint solutions
            success (bool): whether IK computation was successful
        """
        assert pose_matrix.shape == (4, 4), "Expected 4x4 transformation matrix"

        try:
            q_solutions, num_solutions, success = self.rvl_ik_solver.inv_kinematics_all_sols_prev(pose_matrix)

            if not success or num_solutions == 0:
                rospy.logwarn("RVL IK solver failed to find solutions.")
                return None, False

            # Reshape and trim based on number of solutions
            q_solutions = np.array(q_solutions).reshape((8, -1))[:num_solutions]
            q_solutions = q_solutions[~np.isnan(q_solutions).any(axis=1)]
            num_solutions = q_solutions.shape[0]

            if num_solutions < 1:
                rospy.logwarn("RVL IK solver found no valid solutions.")
                return None, False

            q_solutions[:, 0] -= np.pi # compensate for RVL's offset
            # q_solutions[:, 5] -= np.pi # compensate for RVL's offset
            q_solutions[q_solutions > np.pi] -= 2 * np.pi # normalize to [-pi, pi]
            q_solutions[q_solutions < -np.pi] += 2 * np.pi # normalize to [-pi, pi]

            if check_env_collisions:
                valid_solutions = []
                for q in q_solutions:
                    if self.is_state_valid(q.tolist()):
                        valid_solutions.append(q)
                q_solutions = np.array(valid_solutions)
            if check_self_collisions:
                valid_solutions = []
                for q in q_solutions:
                    if self.is_state_self_collision_free(q.tolist()):
                        valid_solutions.append(q)
                q_solutions = np.array(valid_solutions)
            if q_solutions.size == 0:
                rospy.logwarn("No valid IK solutions found after collision checks.")
                return None, False
            return q_solutions, True

        except Exception as e:
            rospy.logerr("RVL IK solver error: %s", str(e))
            return None, False

    def get_closest_ik_solution(self, T_6_0: np.ndarray, joints: Union[np.array, None]) -> np.ndarray:
        """
        Uses the RVL IK solver to get all solutions for the given pose,
        and returns the one closest to the current joint configuration
        using Chebyshev (L∞) distance.

        Args:
            T_G_0 (np.ndarray): 4x4 pose matrix of the end-effector in base frame
            joints (np.array): Array of joint configurations to compare against

        Returns:
            np.ndarray: 6-element joint solution closest to current config
        """
        if T_6_0.shape != (4, 4):
            raise ValueError("Expected a 4x4 transformation matrix for T_6_0")

        # Get all IK solutions using RVL solver
        q_solutions, num_sols, success = self.rvl_ik_solver.inv_kinematics_all_sols_prev(T_6_0)

        if not success or num_sols == 0:
            rospy.logwarn("RVL IK solver found no solutions.")
            return None

        q_solutions = np.array(q_solutions)[:num_sols]
        q_solutions = q_solutions[~np.isnan(q_solutions).any(axis=1)]
        num_sols = q_solutions.shape[0]

        q_solutions[:, 0] -= np.pi
        q_solutions[q_solutions > np.pi] -= (2.0*np.pi)
        q_solutions[q_solutions < -np.pi] += (2.0*np.pi)
        np.unwrap(q_solutions, axis=0)
        
        valid_solutions = []
        for q in q_solutions:
            if self.is_state_self_collision_free(q.tolist()):
                valid_solutions.append(q)
        q_solutions = np.array(valid_solutions)

        if q_solutions.size == 0:
            rospy.logwarn("No valid IK solutions found after self-collision checks.")
            return None
    
        # Get current joint configuration
        q_current = self.get_current_joint_values() if joints is None else joints

        # Compute Chebyshev distance for each solution
        cheb_dists = np.max(np.abs(q_solutions - q_current), axis=1)
        closest_idx = np.argmin(cheb_dists)

        closest_q = q_solutions[closest_idx]

        rospy.loginfo("Found %d IK solutions. Closest (Chebyshev dist: %.4f)", num_sols, cheb_dists[closest_idx])

        return closest_q

    def get_ik_solution_moveit(self, pose_matrix: np.ndarray) -> np.ndarray:
        """
        Computes the inverse kinematics using MoveIt for the given pose matrix.

        Args:
            pose_matrix (np.ndarray): 4x4 transformation matrix representing desired end-effector pose (T_G_0)

        Returns:
            np.ndarray: 6-element array of joint positions [rad] if successful, None otherwise
        """
        assert pose_matrix.shape == (4, 4), "Expected a 4x4 transformation matrix"

        self.group.set_pose_target(matrix_to_pose(pose_matrix))
        (success, joint_trajectory, _, _) = self.group.plan()

        if not success or not joint_trajectory.points:
            rospy.logwarn("MoveIt failed to find IK solution.")
            return None

        joint_values = joint_trajectory.points[0].positions
        return np.array(joint_values)

    def get_fwd_kinematics_moveit(self, joint_values: list) -> np.ndarray:
        from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
        rospy.wait_for_service('/compute_fk')
        fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = 'world'
        fk_request.header.stamp = rospy.Time.now()
        fk_request.fk_link_names = ['tool0']  # or your end-effector link
        joint_state = JointState()
        joint_state.name = self.joint_names
        joint_state.position = joint_values
        fk_request.robot_state = RobotState(joint_state=joint_state)

        try:
            rospy.sleep(0.5)
            response = fk_service(fk_request)
            if response.error_code.val == response.error_code.SUCCESS:
                pose_stamped = response.pose_stamped[0]
                return pose_stamped_to_matrix(pose_stamped)
            else:
                rospy.logerr("FK computation failed with error code: %s", response.error_code.val)
                return None
        except rospy.ServiceException as e:
            rospy.logerr("Service call to /compute_fk failed: %s", e)
            return None

    def get_fwd_kinematics(self, joint_values: np.ndarray) -> np.ndarray:
        T_6_0 = self.rvl_ik_solver.fwd_kinematics_6(joint_values)
        return T_6_0

    # def is_state_valid_slow(self, q: list) -> bool:
    #     """
    #     Validates a single joint configuration using the /check_state_validity service.

    #     Args:
    #         q (list): A single joint configuration [rad].

    #     Returns:
    #         bool: True if the configuration is collision-free, False otherwise.
    #     """

    #     if len(q) != 6:
    #         rospy.logerr("Expected a joint state with 6 values")
    #         return False

    #     req = GetStateValidityRequest()
    #     req.robot_state = RobotState()
    #     req.robot_state.joint_state.name = self.joint_names
    #     req.robot_state.joint_state.position = q
    #     req.group_name = self.group_name

    #     try:
    #         res = self.check_state_validity(req)
    #         return res.valid
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")
    #         return False

    def is_state_valid(self, q: list) -> bool:
        if len(q) != 6:
            rospy.logerr("Expected joint state of length 6")
            return False
        return self.collision_checker.is_state_valid(q)
    
    def is_state_self_collision_free(self, q: list) -> bool:
        """
        Checks if a joint configuration is self-colliding using the FastCollisionChecker.

        Args:
            q (list): A single joint configuration [rad].

        Returns:
            bool: True if the configuration is NOT self-colliding, False otherwise.
        """
        if len(q) != 6:
            rospy.logerr("Expected joint state of length 6")
            return False
        return not self.collision_checker.is_state_self_colliding(q)

    def validate_trajectory_points(self, q_traj: np.ndarray) -> bool:
        """
        Validates each joint state in the trajectory using the /check_state_validity service.

        Args:
            q_traj (np.ndarray): Joint trajectory of shape (N, 6)

        Returns:
            bool: True if all joint states are collision-free, False otherwise.
        """
        from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

        joint_names = self.joint_names

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
            req.group_name = self.group_name

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

    def shutdown(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("UR5Controller shutdown complete.")

    @staticmethod
    def calculate_time_between_points(start, end, max_velocity, max_acceleration):
        position_differences = np.abs(np.array(end) - np.array(start))
        time_velocity = position_differences / max_velocity
        time_acceleration = np.sqrt(2 * position_differences / max_acceleration)
        return max(np.max(time_velocity), np.max(time_acceleration))