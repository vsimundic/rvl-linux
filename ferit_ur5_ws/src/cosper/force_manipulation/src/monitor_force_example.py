import rospy
import numpy as np
from core.real_ur5_controller import UR5Controller
import threading

def monitor_force_and_cancel(robot: UR5Controller, threshold: float = 30.0, check_rate: float = 50.0):
    """
    Monitors force in the background while a trajectory is active.
    Cancels the trajectory if force exceeds the threshold.
    """
    rate = rospy.Rate(check_rate)
    max_wait_cycles = int(2.0 * check_rate)  # 2 seconds timeout

    rospy.loginfo("[monitor] Waiting for trajectory to become ACTIVE...")

    # Phase 1: Wait for the goal to be accepted (state becomes ACTIVE)
    for _ in range(max_wait_cycles):
        state = robot.client.get_state()
        if state == 1:  # ACTIVE
            rospy.loginfo("[monitor] Trajectory is ACTIVE.")
            break
        elif state == 0:  # PENDING
            rate.sleep()
        else:
            rospy.loginfo_throttle(0.5, "[monitor] Goal state: %d (waiting for ACTIVE...)", state)
            rate.sleep()
    else:
        rospy.logwarn("[monitor] Timeout waiting for trajectory to become ACTIVE. Final state: %s", str(state))
        return

    # Phase 2: Monitor force while ACTIVE
    while not rospy.is_shutdown() and robot.client.get_state() == 1:
        wrench = robot.get_current_wrench()
        force = np.linalg.norm(wrench[:3])
        rospy.loginfo_throttle(1.0, "[monitor] Force: %.2f N", force)

        if force > threshold:
            rospy.logwarn("Force threshold exceeded: %.2f N", force)
            robot.force_violation = True
            robot.cancel_trajectory()
            return

        rate.sleep()

    rospy.loginfo("[monitor] Trajectory no longer ACTIVE. Monitoring stopped.")


# Usage example:
robot = UR5Controller()
trajectory = [...]
# Zero the force-torque sensor before starting the trajectory
robot.zero_ft_sensor()
monitor_thread = threading.Thread(target=monitor_force_and_cancel, args=(robot, 5.0))
monitor_thread.start()
success = robot.send_joint_trajectory_action(trajectory, max_velocity=0.5, max_acceleration=0.5)
monitor_thread.join()
# The program will wait for the monitoring thread to finish before proceeding.
