#!/usr/bin/env python

import moveit_commander, geometry_msgs.msg, moveit_msgs.msg, rospy, sys
import roslib
from time import time
from trac_ik_python.trac_ik import IK; 
import sys
import socket
import time

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

from scipy.spatial.transform import Rotation
import numpy as np
import math 

from tf.transformations import quaternion_from_euler,quaternion_matrix, quaternion_from_matrix, euler_from_matrix
import tf

roslib.load_manifest('robotiq_3f_gripper_control')

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

class RobotControl:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)   
        # INVERSE KINEMATICS
        self.ik = IK('base_link', 'tool0', solve_type="Distance"); 



    def setPose(self, x, y, z, quaternion):
        x = float(x)
        y = float(y)
        z = float(z)
        qx, qy, qz, qw = quaternion

        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)
        
        self.move_group.go(goal_joints, wait=True)
        self.move_group.stop()


    def openGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 0
        
        # 3 second delay to let the gripper open
        start_time = time.time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)
            end_time = time.time()
            if float(end_time - start_time) >= 3.0: # 3s
                break

    def closeGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1     # activation
        command.rGTO = 1     # request position
        command.rSPA = 255   # closing speed
        command.rFRA = 200   # force
        command.rATR = 0     # automatic opening if the force is reached (normal = 0)
        command.rMOD = 1     # gripper mode -> 0 = normal, 1 = pinch, 2 = wide, 3 = scissors (needs checking)
        command.rPRA = 200   # 0 = open, 255 = close
        
        # 3 second delay to let the gripper close
        start_time = time.time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)
            end_time = time.time()
            if float(end_time - start_time) >= 3.0: # 3s
                break
    

    def executeTrajectory(self, plan):
        self.move_group.execute(plan, wait=True)


    def getTCPvalues(self):
        #BASE VIEW
        ROBOT_HOST = "192.168.22.14" #localhost
        ROBOT_PORT = 30004
        config_filename = "/home/jana/catkin_ws/src/RTDE/RTDE_Python_Client_Library-2.7.2/examples/control_loop_configuration.xml"

        keep_running = True

        logging.getLogger().setLevel(logging.INFO)

        conf = rtde_config.ConfigFile(config_filename)
        state_names, state_types = conf.get_recipe("state")
        setp_names, setp_types = conf.get_recipe("setp")
        watchdog_names, watchdog_types = conf.get_recipe("watchdog")

        con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        con.connect()
        # get controller version
        con.get_controller_version()
        # setup recipes
        con.send_output_setup(state_names, state_types)
        setp = con.send_input_setup(setp_names, setp_types)
        watchdog = con.send_input_setup(watchdog_names, watchdog_types)

        # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
        watchdog.input_int_register_0 = 0

        # start data synchronization
        if not con.send_start():
            sys.exit()

        # control loop
        move_completed = True
        # receive the current state
        state = con.receive()

        if state is not None:
            pose = state.actual_TCP_pose
            joints = state.actual_q
            print(state.actual_TCP_pose)
            print(state.actual_q)

        # kick watchdog
        con.send(watchdog)
        con.send_pause()
        con.disconnect()

        return pose, joints


def sendURcommand(pose, joints):
    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("Start connected, ", s)
    s.sendall(("movej(get_inverse_kin(p"+ str(pose) +", qnear="+ str(joints)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
    print("End start pose")
    data=s.recv(1024)
    s.close()
    time.sleep(10)


def zeroSensor():
    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    time.sleep(1)
    s.sendall(("zero_ftsensor()" + "\n").encode('utf-8')) 
    time.sleep(2)
    data=s.recv(1024)
    s.close()


def writeToTextFile(positions,joints):
    n=len(positions)
    with open("/home/jana/catkin_ws/src/FT300S_Python/URscript.txt", "w") as f:
        f.write("def door_open():\n")
        #f.write("force_mode(tool_pose(), [1, 1, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.2, 0.2, 0.1, 0.60, 0.60, 0.35])\n")
        for i in range(0,n):
            f.write("movej(get_inverse_kin(p"+ str(positions[i]) +", qnear="+ str(joints[i])+"),a=1.4,v=0.2)\n")
        f.write("end\n")
        f.write("door_open()\n")
        f.close()

def sendURscript():
    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    f = open("/home/jana/catkin_ws/src/FT300S_Python/URscript.txt", "rb")
    l = f.read()
    s.sendall(l)
 
    print("Ended sending urscript!")
    data=s.recv(1024)
    s.close()


def get_next_door_pose(current_pose, door_width, angle_deg):
    TTB = current_pose.copy()
    RTB = TTB[0:3, 0:3].copy()

    b = 0.28
    a = door_width
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    Rz = np.matrix([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    TA_A = np.identity(4)
    TA_A[0:3, 0:3] = Rz.copy()

    TGA = np.matrix([[0, 0, 1, 0],
                     [-1, 0, 0, -a],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])

    RGB = np.matrix([[-1, 0, 0],
                    [0, 0, -1],
                    [0, -1, 0]])

    RGT = np.multiply(np.linalg.inv(RTB), RGB)

    TGT = np.identity(4)
    TGT[0:3, 0:3] = RGT.copy()
    TGT[0:3, 3] = np.array([0, 0, b])

    TT_B = TTB @ TGT @ np.linalg.inv(TGA) @ TA_A @ TGA @ np.linalg.inv(TGT)
    print("\nTT_B:\n", TT_B)

    return TT_B


def get_pose_vector_from_transformation_matrix(mat):
    pose_vec = 6*[0]
    rot_obj_temp = Rotation.from_matrix(mat[0:3, 0:3])
    pose_vec[0:3] = np.asarray(mat[0:3, 3]).reshape(-1)
    pose_vec[3:] = rot_obj_temp.as_euler('xyz').copy()
    return pose_vec


def get_transformation_matrix_from_pose_vector(vec):
    mat = np.identity(4)
    euler_angles = vec[3:].copy()
    rot_obj_temp = Rotation.from_euler('xyz', euler_angles)
    mat[0:3, 0:3] = np.matrix(rot_obj_temp.as_matrix()).copy()
    mat[0:3, 3] = vec[0:3].copy()
    return mat


def sample_door_opening_points_and_joints(RC, start_pose_vec, start_joints, door_width, angle_deg, num_points):
    
    TTB = get_transformation_matrix_from_pose_vector(start_pose_vec)

    poses = []
    joints = []
    prev_poseTB = TTB
    prev_joints = start_joints
    for i in range(0, num_points):

        TT_B = get_next_door_pose(prev_poseTB, door_width, angle_deg)
        pose_vecT_B = get_pose_vector_from_transformation_matrix(TT_B)
        qx, qy, qz, qw = quaternion_from_matrix(TT_B)

        current_joints = None
        while (current_joints is None):
            current_joints = RC.ik.get_ik(prev_joints, 
                                          pose_vecT_B[0], 
                                          pose_vecT_B[1], 
                                          pose_vecT_B[2],
                                          qx, qy, qz, qw)

        poses.append(pose_vecT_B)
        joints.append(current_joints)

        prev_poseTB = pose_vecT_B
        prev_joints = current_joints
    
    return poses, joints


if __name__ == '__main__':
    
    RC = RobotControl()

    # start_pose, start_joints = RC.getTCPvalues()
    
    # Hardcoded point and joint
    start_pose = np.array([-0.3617366700949353, -0.5212725440586028, 0.3861139450603652, 1.4916758533713128, -0.6391382562433204, 0.6156284038303728])
    start_joints = np.array([4.219675064086914, -1.9739564100848597, -1.1635468641864222, 3.119579315185547, -2.0547102133380335, 0.7950859069824219])

    RC.openGripper()
    time.sleep(1)

    # Set the robot in the start pose
    sendURcommand(start_pose, start_joints)
    
    # FT sensor zeroing
    zeroSensor()
    time.sleep(1)
    
    # Close gripper
    RC.closeGripper()
    time.sleep(2)

    poses, joints = sample_door_opening_points_and_joints(RC, start_pose, start_joints, door_width=0.35, angle_deg=10, num_points=3)

    # Write points and joints to a txt file
    writeToTextFile(poses, joints)

    # Start manipulation with force mode
    sendURscript()

    time.sleep(15)
    RC.openGripper()
