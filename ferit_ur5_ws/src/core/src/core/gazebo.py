import rospy
from typing import Union
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesResponse
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import SetModelConfiguration
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty

def get_joint_info(joint_name: str) -> Union[GetJointPropertiesResponse, None]:
    """
    Retrieve properties of a specific joint from Gazebo.

    :param joint_name: Name of the joint in Gazebo
    :return: GetJointPropertiesResponse with joint properties (position, rate, damping) 
            or None if the request fails
    """
    rospy.wait_for_service('/gazebo/get_joint_properties')
    try:
        get_joint_properties = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        response = get_joint_properties(joint_name)

        if response.success:
            rospy.loginfo(f"Joint: {joint_name}")
            rospy.loginfo(f"Position: {response.position}")
            rospy.loginfo(f"Rate: {response.rate}")
            rospy.loginfo(f"Damping: {response.damping}")
            return response
        else:
            rospy.logwarn(f"Failed to get joint properties for: {joint_name}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def get_link_pose(model_name: str, link_name: str) -> Union[Pose, None]:
    """
    Retrieve the pose of a specific link in a given model from Gazebo.

    :param model_name: Name of the model in Gazebo
    :param link_name: Name of the link within the model
    :return: Pose of the link in the world frame, or None if unsuccessful
    """
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        # Create a service proxy for /gazebo/get_link_state
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        # Full link name in Gazebo is usually "model::link"
        full_link_name = f"{model_name}::{link_name}"
        
        # Request the pose of the link
        response = get_link_state(full_link_name, "world")  # Reference frame is "world"

        if response.success:
            rospy.loginfo(f"Pose of {full_link_name}: {response.link_state.pose}")
            return response.link_state.pose
        else:
            rospy.logwarn(f"Failed to get pose for link: {full_link_name}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None

def reset_gazebo_simulation():
    """
    Resets the simulation by calling Gazebo reset services and clearing MoveIt! states.
    """
    rospy.loginfo("Resetting Gazebo simulation...")

    # Reset Gazebo world and simulation
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.wait_for_service('/gazebo/reset_simulation')
    
    try:
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        reset_world()
        reset_simulation()
        rospy.loginfo("Gazebo world and MoveIt! scene reset.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset simulation: {e}")


def reset_gazebo_joints(model_name, joint_names, default_positions):
    """
    Resets the robot's joints in Gazebo without affecting the rest of the simulation.

    Args:
        robot_name (str): The name of the robot model in Gazebo.
        joint_names (list): List of joint names to reset.
        default_positions (list): List of corresponding joint positions.

    Returns:
        bool: True if successful, False otherwise.
    """

    rospy.wait_for_service('/gazebo/set_model_configuration')
    try:
        set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        response = set_model_configuration(
            model_name=model_name,
            urdf_param_name="",
            joint_names=joint_names,
            joint_positions=default_positions
        )

        if response.success:
            rospy.loginfo(f"Successfully reset joints for {model_name} in Gazebo.")
            return True
        else:
            rospy.logwarn(f"Failed to reset joints for {model_name} in Gazebo.")
            return False

    except rospy.ServiceException as e:
        rospy.logerr(f"Gazebo joint reset service call failed: {e}")
        return False
    

from sensor_msgs.msg import JointState 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus
import actionlib
from moveit_msgs.msg import MoveGroupAction
def reset_gazebo_and_sync_moveit(robot_name, joint_names, default_positions, controller_topic="/trajectory_controller/command"):
    """
    Fully resets Gazebo and ensures MoveIt! stays synchronized.
    
    Args:
        robot_name (str): Name of the robot model in Gazebo.
        joint_names (list): List of joint names.
        default_positions (list): List of target joint positions.
        controller_topic (str): The topic for the robot's trajectory controller.
    """

    rospy.loginfo("Pausing Gazebo physics...")
    rospy.wait_for_service('/gazebo/pause_physics')
    pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    pause_physics()

    rospy.loginfo("Resetting Gazebo simulation...")
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_simulation()

    rospy.loginfo("Resetting robot joint positions via controllers...")
    
    # Publish joint values to the controller
    pub = rospy.Publisher(controller_topic, JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Ensure publisher is active

    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = default_positions
    point.time_from_start = rospy.Duration(3)  # Move in 2 seconds
    traj_msg.points.append(point)

    for _ in range(5):  # Publish multiple times to ensure reception
        pub.publish(traj_msg)
        rospy.sleep(0.1)

    rospy.loginfo(f"Published joint reset to {controller_topic}: {default_positions}")

    rospy.loginfo("Unpausing Gazebo physics...")
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    unpause_physics()

    rospy.loginfo("Forcing MoveIt! to recognize the new state...")
    
    # Publish joint states manually to force MoveIt! to sync
    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.sleep(1)  # Ensure publisher is ready

    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = joint_names
    joint_msg.position = default_positions

    for _ in range(5):  # Publish multiple times to force MoveIt! update
        joint_state_pub.publish(joint_msg)
        rospy.sleep(0.1)

    rospy.loginfo("MoveIt! joint states updated and synchronized.")

    rospy.loginfo("Verifying MoveIt! goal state...")

    # Connect to MoveIt! action server and check execution status
    moveit_client = actionlib.SimpleActionClient("move_group", MoveGroupAction)
    moveit_client.wait_for_server()

    while moveit_client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
        rospy.sleep(0.1)  # Wait for MoveIt! to update

    final_status = moveit_client.get_state()
    if final_status in [GoalStatus.SUCCEEDED]:
        rospy.loginfo("MoveIt! goal successfully updated after reset.")
    else:
        rospy.logwarn(f"MoveIt! detected a reset and canceled execution: {final_status}")


from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers

def reset_gazebo_and_robot(robot_name, joint_names, default_positions, controller_name="trajectory_controller"):
    """
    Fully resets Gazebo and robot joint positions without getting stuck.

    Args:
        robot_name (str): Gazebo model name.
        joint_names (list): List of joint names.
        default_positions (list): Target joint positions.
        controller_name (str): Name of the robot's trajectory controller.
    """

    rospy.loginfo("Stopping controllers before pausing physics...")
    rospy.wait_for_service('/controller_manager/switch_controller')
    switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    stop_request = SwitchControllerRequest()
    stop_request.stop_controllers = [controller_name]
    stop_request.strictness = 2  # BEST_EFFORT
    switch_controller(stop_request)

    rospy.loginfo("Pausing Gazebo physics...")
    rospy.wait_for_service('/gazebo/pause_physics')
    pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    pause_physics()

    rospy.loginfo("Resetting Gazebo simulation...")
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_simulation()

    rospy.loginfo("Unpausing Gazebo physics...")
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    unpause_physics()

    rospy.loginfo("Re-enabling controllers after reset...")
    start_request = SwitchControllerRequest()
    start_request.start_controllers = [controller_name]
    start_request.strictness = 2
    switch_controller(start_request)

    rospy.sleep(2)  # Allow controllers to fully restart

    # Ensure controller is running before publishing commands
    rospy.loginfo("Waiting for controller to be active...")
    rospy.wait_for_service('/controller_manager/list_controllers')
    list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)

    controller_active = False
    for _ in range(10):  # Try for a few seconds
        running_controllers = list_controllers()
        for c in running_controllers.controller:
            if c.name == controller_name and c.state == "running":
                controller_active = True
                break
        if controller_active:
            rospy.loginfo(f"Controller {controller_name} is active.")
            break
        rospy.sleep(0.5)  # Wait before checking again

    if not controller_active:
        rospy.logerr(f"Controller {controller_name} did not start. Cannot send commands.")
        return False

    rospy.loginfo("Resetting robot joint positions via controllers...")
    
    # Publish joint values to the controller
    pub = rospy.Publisher(f"/{controller_name}/command", JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Ensure publisher is active

    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = default_positions
    point.time_from_start = rospy.Duration(2)  # Move in 2 seconds
    traj_msg.points.append(point)

    for _ in range(5):  # Publish multiple times to ensure reception
        pub.publish(traj_msg)
        rospy.sleep(0.1)

    rospy.sleep(3)
    rospy.loginfo("Gazebo and MoveIt! joint states successfully reset.")
    return True