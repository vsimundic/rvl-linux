import rospy
import numpy as np
from core.real_ur5_controller import UR5Controller
from gazebo_push_open.cabinet_model import Cabinet
from core.transforms import rot_z, rot_y
from core.util import get_nearest_joints
import RVLPYDDManipulator as rvlpy

def chebyshev_distance(q1, q2):
    q1 = np.array(q1)
    q2 = np.array(q2)
    error = np.abs(q1 - q2)
    max_error = np.max(error, axis=0)
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

# def monitor_tactile_force_and_cancel_on_drop(
#     robot: UR5Controller,
#     drop_threshold: float = 10.0,
#     near_zero_threshold: float = 2.0,
#     check_rate: float = 50.0
# ):
    

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

def monitor_tactile_contact_establish(
    robot: UR5Controller,
    contact_threshold: float = 1.0,
    contact_timeout: float = 2.0,
    check_rate: float = 50.0
):
    """
    Monitors tactile sensor data per sensor and taxel, until the contact is established or timeout occurs.
    Cancels trajectory if timeout occurs.
    """
    rate = rospy.Rate(check_rate)
    max_wait_cycles = int(2 * check_rate)
    rospy.loginfo("[monitor-tactile] Waiting for trajectory to become ACTIVE...")
    for _ in range(max_wait_cycles):
        state = robot.client.get_state()
        if state == 1:
            break
        rate.sleep()
    else:
        rospy.logwarn("[monitor-tactile] Timeout waiting for ACTIVE goal.")
        return
    rospy.loginfo("[monitor-tactile] Monitoring tactile sensors per taxel...")
    max_timeout_cycles = int(contact_timeout * check_rate)
    current_timeout_cycles = 0
    while not rospy.is_shutdown() and robot.client.get_state() == 1 and current_timeout_cycles < max_timeout_cycles:
        if robot.tactile_data is not None:
            force_magnitudes = np.linalg.norm(robot.tactile_data, axis=2)
            max_force = np.max(force_magnitudes)
            rospy.loginfo_throttle(1.0, "[monitor-tactile] Max tactile force: %.3f", max_force)
            if max_force > contact_threshold:
                rospy.loginfo("Contact established on at least one taxel (max %.3f > %.3f).", max_force, contact_threshold)
                robot.tactile_contact_established = True
                return
            current_timeout_cycles += 1
        rate.sleep()

    rospy.logwarn("Trajectory timeout. Cancelling trajectory.")
    robot.cancel_trajectory()

def monitor_tactile_loss_and_remember_joints(
    robot: UR5Controller,
    remembered_joints: list,
    contact_threshold: float = 1.0,
    contact_establishment_timeout: float = 2.5,
    check_rate: float = 50.0
):
    """
    Monitors tactile sensor data per sensor and taxel, cancels trajectory if any sensor loses contact.
    Remembers the joint configuration when loss is detected.

    Args:
        robot (UR5Controller): controller instance
        remembered_joints (list): list to store joint values on contact loss
        contact_threshold (float): threshold for considering contact lost per taxel
        contact_establishment_timeout (float): timeout for establishing contact
        check_rate (float): Hz rate to check tactile data
    """
    rate = rospy.Rate(check_rate)
    max_wait_cycles = int(2.0 * check_rate)
    max_timeout_cycles = int(contact_establishment_timeout * check_rate)
    current_timeout_cycles = 0

    rospy.loginfo("[monitor-tactile] Waiting for trajectory to become ACTIVE...")

    for _ in range(max_wait_cycles):
        state = robot.client.get_state()
        if state == 1:
            break
        rate.sleep()
    else:
        rospy.logwarn("[monitor-tactile] Timeout waiting for ACTIVE goal.")
        return

    rospy.loginfo("[monitor-tactile] Monitoring tactile sensors per taxel...")

    while not rospy.is_shutdown() and robot.client.get_state() == 1:

        if not robot.tactile_contact_established:
            if current_timeout_cycles >= max_timeout_cycles:
                rospy.logwarn("Contact not established on any taxel. Cancelling trajectory.")
                q = robot.get_current_joint_values()
                remembered_joints.append(q.tolist())
                robot.cancel_trajectory()
                return
            else:
                rospy.loginfo_throttle(1.0, "[monitor-tactile] Waiting for contact to be established...")
                continue
        
        if robot.tactile_contact_established and robot.tactile_data is not None:
            force_magnitudes = np.linalg.norm(robot.tactile_data, axis=2)  # shape (n_sensors, n_taxels, 3)
            max_force = np.max(force_magnitudes)
            rospy.loginfo_throttle(1.0, "[monitor-tactile] Max tactile force: %.3f", max_force)

            if max_force < contact_threshold:
                robot.tactile_contact_established = False
                rospy.logwarn("Contact lost on at all taxels (max %.3f < %.3f). Cancelling.", max_force, contact_threshold)
                q = robot.get_current_joint_values()
                remembered_joints.append(q.tolist())
                robot.cancel_trajectory()
                return

        current_timeout_cycles += 1
        rate.sleep()

    rospy.loginfo("[monitor-tactile] Trajectory finished. Exiting tactile monitor.")


def get_camera_pose_on_sphere_distance(cabinet_model: Cabinet, 
                                       robot: UR5Controller,
                                       rvl_manipulator: rvlpy.PYDDManipulator, 
                                       T_6_0_capture: np.ndarray, lost_contact_estimated_state: float):
    T_Arot_A = np.eye(4)
    T_Arot_A[:3, :3] = rot_z(np.deg2rad(lost_contact_estimated_state))
    
    T_A_W = cabinet_model.T_A_S.copy()

    T_A_W_rot = T_A_W.copy()

    T_A_W = T_A_W @ T_Arot_A

    # Door center
    T_B_A = np.eye(4)
    T_B_A[:3, 3] = np.array([cabinet_model.rx - cabinet_model.d_door*0.5, cabinet_model.ry, 0])
    # T_B_A[:3, 3] = np.array([-cabinet_model.d_door*0.5, -cabinet_model.w_door * 0.5, 0]) 
    T_B_W_ = T_A_W @ T_B_A

    # Rotations of T_B_W around the y axis
    angles_y = np.linspace(np.deg2rad(20.), np.pi/2, 20)
    angles_z = np.linspace(np.deg2rad(-45.), np.deg2rad(45.), 11) 
    Ty = np.eye(4)
    Tz = np.eye(4)
    T_C_W_new = np.eye(4)
    T_B_W_remember = T_B_W_.copy()
    joints_saved, n_sol_saved = [], 0
    T_6_0_new = np.eye(4)

    for angle_y in range(len(angles_y)):
        Ty[:3,:3] = rot_y(angles_y[angle_y])
        for angle_z in range(len(angles_z)):
            Tz[:3,:3] = rot_z(angles_z[angle_z])
            
            T_B_W = T_B_W_ @ Ty @ Tz

            T_C_W_capture = robot.T_0_W @ T_6_0_capture @ robot.T_C_6
            capture_dist = np.linalg.norm(T_C_W_capture[:3, 3] - T_B_W[:3, 3])
            
            t_C_B = np.array([-capture_dist, 0, 0])

            T_C_W_new[:3, 2] = T_B_W[:3, 0].copy()
            T_C_W_new[:3, 0] = np.cross(T_C_W_new[:3, 2], robot.T_0_W[:3, 2])
            T_C_W_new[:3, 1] = np.cross(T_C_W_new[:3, 2], T_C_W_new[:3, 0])
            T_C_W_new[:3, 3] = T_B_W[:3, :3] @ t_C_B + T_B_W[:3, 3]

            T_6_0_new = np.linalg.inv(robot.T_0_W) @ T_C_W_new @ np.linalg.inv(robot.T_C_6)

            """ # Visualization
            if True:
                # Visualization
                geoms = []
                origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
                geoms.append(origin_rf)
                # origin_rf.paint_uniform_color([1, 0, 0])
                T_A_W_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                T_A_W_mesh.transform(T_A_W)
                geoms.append(T_A_W_mesh)
                T_A_W_rot_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                T_A_W_rot_mesh.transform(T_A_W_rot)
                geoms.append(T_A_W_rot_mesh)
                T_B_W_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                T_B_W_mesh.transform(T_B_W)
                geoms.append(T_B_W_mesh)
                T_C_W_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                T_C_W_mesh.transform(T_C_W_capture)
                geoms.append(T_C_W_mesh)
                T_C_W_mesh.paint_uniform_color([0, 1, 0])
                T_C_W_new_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
                T_C_W_new_mesh.transform(T_C_W_new)
                geoms.append(T_C_W_new_mesh)
                # T_C_W_new_mesh.paint_uniform_color([0, 0, 1])
                cabinet_model.create_mesh()
                dd_plate_mesh = cabinet_model.dd_plate_mesh
                dd_plate_mesh_rot = copy.deepcopy(dd_plate_mesh)
                T_DD_W = T_A_W @ cabinet_model.T_D_A
                T_O_D =  np.linalg.inv(cabinet_model.T_D_A) @ np.linalg.inv(cabinet_model.T_A_O_init)
                dd_plate_mesh.transform(T_O_D)
                dd_plate_mesh.transform(T_DD_W)
                dd_plate_mesh.compute_vertex_normals()
                geoms.append(dd_plate_mesh)
                T_DD_W_rot = T_A_W_rot @ cabinet_model.T_D_A
                dd_plate_mesh_rot.transform(T_O_D)
                dd_plate_mesh_rot.transform(T_DD_W_rot)
                geoms.append(dd_plate_mesh_rot)
                o3d.visualization.draw_geometries(geoms, window_name="Door model", width=1920, height=1080, left=50, top=50, mesh_show_back_face=True)
            """
            
            joints, n_sol, _ = rvl_manipulator.inv_kinematics_all_sols_prev(T_6_0_new)
            
            if n_sol > 0:
                joints[:, 0] -= np.pi
                # joints[:, 5] -= np.pi
                joints[joints > np.pi] -= 2 * np.pi
                joints[joints < -np.pi] += 2 * np.pi

                for i_sol in range(n_sol):
                    q = joints[i_sol]

                    if q[2] > np.pi*0.5:
                        continue
                    if q[2] < -np.pi*0.25:
                        continue
                    
                    if robot.is_state_valid(q.tolist()):
                        joints_saved.append(q.tolist())

                # if len(joints_saved) > 0:
                #     break

    # if len(joints_saved) > 0:
    #     current_q = robot.get_current_joint_values()
    #     current_q = np.array(current_q)
    #     joints_saved = np.array(joints_saved)
        
    #     q_nearest, idx_nearest = get_nearest_joints(joints_saved, current_q)
    #     T_6_0_nearest = robot.get_fwd_kinematics_moveit(q_nearest.tolist())
    #     T_C_W_new = robot.T_0_W @ T_6_0_nearest @ robot.T_C_6
    #     joints_saved = q_nearest.copy()
    #     return T_C_W_new, joints_saved
    
    # Find closest cheb dist
    if len(joints_saved) > 0:
        current_q = np.array(robot.get_current_joint_values())
        joints_saved = np.array(joints_saved)

        cheb_dists = [chebyshev_distance(q, current_q) for q in joints_saved]
        idx_nearest = np.argmin(cheb_dists)
        q_nearest = joints_saved[idx_nearest]

        T_6_0_nearest = robot.get_fwd_kinematics_moveit(q_nearest.tolist())
        T_C_W_new = robot.T_0_W @ T_6_0_nearest @ robot.T_C_6
        joints_saved = q_nearest.copy()
        return T_C_W_new, joints_saved
    
    else:
        rospy.logwarn("No valid joint configurations found.")
        return None, None
