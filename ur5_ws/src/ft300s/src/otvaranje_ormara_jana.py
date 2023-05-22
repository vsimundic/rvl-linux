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

import numpy as np
import math 

from tf.transformations import quaternion_from_euler

roslib.load_manifest('robotiq_3f_gripper_control')

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

class RobotControl:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)

        # definirati instancu
        self.robot = moveit_commander.RobotCommander()

        # definirati scenu
        self.scene = moveit_commander.PlanningSceneInterface()

        # stvoriti objekt koji pruza sucelje za planiranje pomaka za definiranu grupu u setup assistant-u
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # gripper publisher
        self.pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)
        
        #INVERSE KINEMATICS
        self.ik = IK('base_link', 'tool0', solve_type="Distance"); 



    def setPosition(self, x, y, z):

        # postavljanje orijentacije zadnjeg clanka
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = 0.66259
        # pose_goal.orientation.y = 0.24732
        # pose_goal.orientation.z = -0.6532
        # pose_goal.orientation.w = -0.27055

        # postavljanje pozicije
        # pose_goal.position.x = float(x)
        # pose_goal.position.y = float(y)
        # pose_goal.position.z = float(z)
        
        # pose_goal.position.x = -0.2
        # pose_goal.position.y = 0.5
        # pose_goal.position.z = 0.5
        
        

        # planiranje trajektorije
        #self.move_group.set_pose_target(pose_goal)
        #plan = self.move_group.plan()[1]
        #return plan

        x = float(x)
        y = float(y)
        z = float(z)

        x = 0.2
        y = 0.4
        z = 0.7

        #gripper faces down
        qx = 1
        qy = 0
        qz = 0
        qw = 0

        current_joints = self.move_group.get_current_joint_values()
        goal_joints = self.ik.get_ik(current_joints, x, y, z, qx, qy, qz, qw)

        print("Inverse kin:")
        print(goal_joints)

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = goal_joints[0]
        joint_goal[1] = goal_joints[1]
        joint_goal[2] = goal_joints[2]
        joint_goal[3] = goal_joints[3]
        joint_goal[4] = goal_joints[4]
        joint_goal[5] = goal_joints[5]

            
        self.move_group.go(joint_goal, wait=True)

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
        
        
        # Delay od 3 sekunde da se saka otvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
                break

    def closeGripper(self):

        command = Robotiq3FGripperRobotOutput()
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150
        command.rATR = 0
        command.rMOD = 1
        command.rPRA = 100
        
        # Delay od 3 sekunde da se saka zatvori
        start_time = time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time()

            if float(end_time - start_time) >= 3.0:
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
            pozicija = state.actual_TCP_pose
            joints = state.actual_q
            print(state.actual_TCP_pose)
            print(state.actual_q)


        # kick watchdog
        con.send(watchdog)

        con.send_pause()

        con.disconnect()
        return pozicija, joints

    def calculateNextStep(self, pozicija):

        sizeOfTheDoor = 0.35 #m
        openDoorFor = 0.2085 #m
        angleOpenDoor = 22.5 #° ---> PREBACI U RADIJANE

        angleOpenDoor = math.radians(angleOpenDoor)
        #print("angleOpenDoor: ", angleOpenDoor)

        alfa = np.tan(openDoorFor / sizeOfTheDoor) #stvarni kut prvog trokuta
        #print("ALFA: ", alfa)

        a = openDoorFor * np.cos(angleOpenDoor) ###y
        b = openDoorFor * np.sin(angleOpenDoor) ### x

        #MORA SE MNOZIT S MINUS JEDAN JER JE RVIZ PLANIRANJE SUPROTNI KOORDINATNI SUSTAV OD OVOG
        pozicija[0] = -1*(b + pozicija[0])
        pozicija[1] = -1*(a + pozicija[1])


        return pozicija


        

if __name__ == '__main__':
    
    # instanca upravljaca robotom

    RC = RobotControl()
    # plan=RC.setPosition(-0.4, -0.4, 0.8)
    # RC.executeTrajectory(plan)

    #current_joints = RC.move_group.get_current_joint_values() #move joint vrijednosti iz polyscopa
    #print(current_joints)

    poz, joints = RC.getTCPvalues()
    print("tip poz: ", type(poz))
    print("star_poz: ", poz)

    novi_poz = RC.calculateNextStep(poz)
    print("novi_poz: ", novi_poz)

    quaternion = quaternion_from_euler(novi_poz[3], novi_poz[4], novi_poz[5])
    qx,qy,qz,qw = quaternion

    goal_joints = RC.ik.get_ik(joints, novi_poz[0], novi_poz[1], novi_poz[2], qx, qy, qz, qw)

    print("Stari: ", joints)
    print("GOAL:", goal_joints)
    goal_joints = list(goal_joints)

    print("GOAL:", goal_joints)

    for i in range(0,len(goal_joints)):
        print(np.degrees(goal_joints[i]))

    # joint_goal = RC.move_group.get_current_joint_values()
    # joint_goal[0] = goal_joints[0]
    # joint_goal[1] = goal_joints[1]
    # joint_goal[2] = goal_joints[2]
    # joint_goal[3] = goal_joints[3]
    # joint_goal[4] = goal_joints[4]
    # joint_goal[5] = goal_joints[5]

    # RC.move_group.set_joint_value_target(joint_goal)
    # plan = RC.move_group.plan()

    # RC.move_group.stop()




    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("spojen, ", s)

    #novi_poz = [-0.1714525672191185, -0.5752180357905977, 0.3990368460928424, 1.450011982914319, -0.5325162737000843, 0.5892626994523873]
    #goal_joints = [4.588145732879639, -1.8396227995501917, -1.3286092917071741, 3.2249526977539062, -1.7115009466754358, 0.7316369414329529]

    #MORA SE PONISTIT OKRETANJE KORDINARNOG SUSTAVA IZ RVIZA , JER SE OVDJE SALJE STVARNI KOORDINATNI SUSTAV ROBOTA
    novi_poz[0] *= -1
    novi_poz[1] *= -1
  
    #s.sendall(("movej(get_inverse_kin(p"+ str(novi_poz) +", qnear="+ str(goal_joints)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
 
    print("End program")
    data=s.recv(1024)
    s.close()




    ##DRUGA TOČKA
    novi_poz2 = RC.calculateNextStep(novi_poz)
    print("\n novi_poz BROJ 2: ", novi_poz2)

    quaternion2 = quaternion_from_euler(novi_poz2[3], novi_poz2[4], novi_poz2[5])
    qx,qy,qz,qw = quaternion2

    goal_joints2 = RC.ik.get_ik(goal_joints, novi_poz2[0], novi_poz2[1], novi_poz2[2], qx, qy, qz, qw)
    goal_joints2 = list(goal_joints2)

    print("GOAL:", goal_joints2)

    HOST = "192.168.22.14"
    PORT = 30002
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("spojen, ", s)

    #novi_poz = [-0.1714525672191185, -0.5752180357905977, 0.3990368460928424, 1.450011982914319, -0.5325162737000843, 0.5892626994523873]
    #goal_joints = [4.588145732879639, -1.8396227995501917, -1.3286092917071741, 3.2249526977539062, -1.7115009466754358, 0.7316369414329529]

    #MORA SE PONISTIT OKRETANJE KORDINARNOG SUSTAVA IZ RVIZA , JER SE OVDJE SALJE STVARNI KOORDINATNI SUSTAV ROBOTA
    novi_poz2[0] *= -1
    novi_poz2[1] *= -1
  
    s.sendall(("movej(get_inverse_kin(p"+ str(novi_poz2) +", qnear="+ str(goal_joints2)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
 
    print("End program")
    data=s.recv(1024)
    s.close()






    # Tocka 1
    #RC.setPosition(-0.4, -0.40, 0.8)
    #RC.executeTrajectory(plan)
    print("Finished point 1.")

    # # Otvori gripper
    #RC.openGripper()

    # Zatvori gripper
    #RC.closeGripper()

    # Tocka 2
    # plan = RC.setPosition(-0.3, -0.30, 0.8)
    # RC.executeTrajectory(plan)
    # print("Finished point 2.")


    # Otvori gripper
    #RC.openGripper()

    print("Sequence done!")

    rospy.spin()
 

#[-0.1714525672191185, -0.5752180357905977, 0.3990368460928424, 1.450011982914319, -0.5325162737000843, 0.5892626994523873] #---> mm, rad
#[4.588145732879639, -1.8396227995501917, -1.3286092917071741, 3.2249526977539062, -1.7115009466754358, 0.7316369414329529] #--->joints
