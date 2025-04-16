import rospy
import numpy as np
from core.real_ur5_controller import UR5Controller

def chebyshev_distance(q1, q2):
    q1 = np.array(q1)
    q2 = np.array(q2)
    error = np.abs(q1 - q2)
    max_error = np.max(error)
    return max_error

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

def monitor_force_and_cancel_on_drop(
    robot: UR5Controller,
    drop_threshold: float = 10.0,
    near_zero_threshold: float = 2.0,
    check_rate: float = 50.0
):
    """
    Monitors force and cancels the trajectory if the force drops significantly
    or becomes near zero.

    Args:
        robot (UR5Controller): controller instance
        drop_threshold (float): cancel if force drops by this much (ΔN)
        near_zero_threshold (float): cancel if force drops below this value
        check_rate (float): rate (Hz) at which to check force
    """
    rate = rospy.Rate(check_rate)
    max_wait_cycles = int(2.0 * check_rate)

    rospy.loginfo("[monitor-drop] Waiting for trajectory to become ACTIVE...")

    # Wait for the goal to be accepted
    for _ in range(max_wait_cycles):
        state = robot.client.get_state()
        if state == 1:  # ACTIVE
            break
        elif state == 0:  # PENDING
            rate.sleep()
        else:
            rospy.loginfo_throttle(0.5, "[monitor-drop] Goal state: %d", state)
            rate.sleep()
    else:
        rospy.logwarn("[monitor-drop] Timeout waiting for ACTIVE goal.")
        return

    rospy.loginfo("[monitor-drop] Trajectory ACTIVE. Monitoring force...")

    initial_force = np.linalg.norm(robot.get_current_wrench()[:3])
    prev_force = initial_force

    while not rospy.is_shutdown() and robot.client.get_state() == 1:
        wrench = robot.get_current_wrench()
        force_mag = np.linalg.norm(wrench[:3])
        delta = prev_force - force_mag
        prev_force = force_mag

        rospy.loginfo_throttle(1.0, "[monitor-drop] Force: %.2f N, delta: %.2f N", force_mag, delta)

        if force_mag < near_zero_threshold:
            rospy.logwarn("Force near zero: %.2f N. Cancelling trajectory.", force_mag)
            robot.force_violation = True
            robot.cancel_trajectory()
            return

        if delta >= drop_threshold:
            rospy.logwarn("Force dropped significantly (Δ %.2f N). Cancelling trajectory.", delta)
            robot.force_violation = True
            robot.cancel_trajectory()
            return

        rate.sleep()

    rospy.loginfo("[monitor-drop] Trajectory finished. Exiting monitor.")

def monitor_force_drop_and_remember_joints(
    robot: UR5Controller,
    force_drop_joints: list,
    drop_threshold: float = 10.0,
    near_zero_threshold: float = 2.0,
    check_rate: float = 50.0
) -> None:
    """
    Monitors force during active trajectory and remembers joint configurations
    when force drops significantly or goes near zero.

    Appends joint values to robot.force_drop_joints.
    Returns the full list after monitoring ends.
    """
    rate = rospy.Rate(check_rate)
    max_wait_cycles = int(2.0 * check_rate)

    rospy.loginfo("[monitor+remember] Waiting for trajectory to become ACTIVE...")

    for _ in range(max_wait_cycles):
        state = robot.client.get_state()
        if state == 1:  # ACTIVE
            break
        elif state == 0:  # PENDING
            rate.sleep()
        else:
            rospy.loginfo_throttle(0.5, "[monitor+remember] Goal state: %d", state)
            rate.sleep()
    else:
        rospy.logwarn("[monitor+remember] Timeout waiting for ACTIVE goal.")
        return []

    rospy.loginfo("[monitor+remember] Trajectory ACTIVE. Monitoring force...")

    force_dropped = False
    prev_force = np.linalg.norm(robot.get_current_wrench()[:3])

    while not rospy.is_shutdown() and robot.client.get_state() == 1:
        wrench = robot.get_current_wrench()
        force = np.linalg.norm(wrench[:3])
        delta = prev_force - force
        prev_force = force

        rospy.loginfo_throttle(1.0, "[monitor+remember] Force: %.2f N, delta: %.2f N", force, delta)

        if not force_dropped and (force < near_zero_threshold or delta > drop_threshold):
            force_dropped = True
            q = robot.get_current_joint_values()
            force_drop_joints.append(q.tolist())
            rospy.logwarn("Remembering joint config due to force drop.")
            break

        rate.sleep()
