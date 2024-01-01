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

from tf.transformations import quaternion_from_euler,quaternion_matrix, quaternion_from_matrix, euler_from_matrix
import tf

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
        start_time = time.time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time.time()

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
        start_time = time.time()
        
        while True:
            self.pub.publish(command)
            rospy.sleep(0.1)

            end_time = time.time()

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
        openDoorFor = 0.10 #0.2085 #m
        angleOpenDoor = 10 #22.5 #° ---> PREBACI U RADIJANE

        angleOpenDoor = math.radians(angleOpenDoor)
        #print("angleOpenDoor: ", angleOpenDoor)

        alfa = np.tan(openDoorFor / sizeOfTheDoor) #stvarni kut prvog trokuta
        #print("ALFA: ", alfa)

        a = openDoorFor * np.cos(angleOpenDoor) ###y
        b = openDoorFor * np.sin(angleOpenDoor) ### x
        print("Zelim pomak za a: ", a)
        print("Zelim pomak za b: ", b)

        #MORA SE MNOZIT S MINUS JEDAN JER JE RVIZ PLANIRANJE SUPROTNI KOORDINATNI SUSTAV OD OVOG
        pozicija[0] = -1*(b + pozicija[0])
        pozicija[1] = -1*(a + pozicija[1])
        print("-------------P[0]: ", pozicija[0])
        print("-------------P[1]: ", pozicija[1])

        return pozicija
    
    def sendURcommand(pozition,joints):
        HOST = "192.168.22.14"
        PORT = 30002
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        s.connect((HOST,PORT))
        print("spojen start, ", s)
        s.sendall(("movej(get_inverse_kin(p"+ str(pozition) +", qnear="+ str(joints)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
        print("End start poz")
        data=s.recv(1024)
        s.close()
        time.sleep(5)
    
def writeToTextFile(positions,joints):
    n=len(positions)
    with open("URscript.txt", "w") as f:
        f.write("def otvaranjeOrmara():\n")
        f.write("force_mode(tool_pose(), [1, 1, 0, 0, 0, 0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 2, [0.2, 0.2, 0.1, 0.35, 0.35, 0.35])\n")
        for i in range(0,n):
            f.write("movej(get_inverse_kin(p"+ str(positions[i]) +", qnear="+ str(joints[i])+"),a=1.4,v=0.2)\n")
        f.write("end\n")
        f.write("otvaranjeOrmara()\n")
        f.close()



        

if __name__ == '__main__':
    
    RC = RobotControl()

    HOST = "192.168.22.14"
    PORT = 30002

    #postavljanje robota u pocetnu poziciju
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("spojen start, ", s)
    start_tocka = [-0.3617366700949353, -0.5212725440586028, 0.3861139450603652, 1.4916758533713128, -0.6391382562433204, 0.6156284038303728]
    start_joints = [4.219675064086914, -1.9739564100848597, -1.1635468641864222, 3.119579315185547, -2.0547102133380335, 0.7950859069824219]
    s.sendall(("movej(get_inverse_kin(p"+ str(start_tocka) +", qnear="+ str(start_joints)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
    print("End start poz")
    data=s.recv(1024)
    s.close()

    time.sleep(5)
 
   
    poz, joints = RC.getTCPvalues()
    print("poz: ", poz)

    quaternion = quaternion_from_euler(poz[3], poz[4], poz[5])
    #qx_,qy_,qz_,qw_ = quaternion

    theta = np.sqrt(poz[3]**2+ poz[4]**2+ poz[5]**2) 
    qx = poz[3]* (math.sin(theta/2)/theta)
    qy = poz[4]* (math.sin(theta/2)/theta)
    qz = poz[5]* (math.sin(theta/2)/theta)
    qw = math.cos(theta/2)



    bTt = quaternion_matrix([qx,qy,qz, qw])
    #matrix2 = quaternion_matrix([qw_,qx_,qy_,qz_])
    #print("matrix2: ")
    #print(matrix2)
    print("quaternion matrix(bTt): ")
    print(bTt)
    print("\n")
    bRt = np.array(bTt[0:3,0:3])
    print("bRt: \n",bRt)

    bRg = np.array([[-1, 0, 0],[0, 0, -1],[0, -1, 0]])
    print("bRg: \n", bRg)
    tRb = np.linalg.inv(bRt)
    print("tRb: \n", tRb)

    tRg = np.matmul(tRb,bRg)
    print("tRg: \n", tRg)

    tTg = np.append(tRg,np.array([[0],[0],[0.28]]) , axis = 1) 
    tTg = np.append(tTg, np.array([[0,0,0,1]]), axis = 0)
    print("tTg: \n", tTg)

    aTg = np.array([[0, 0, 1, 0], [-1, 0, 0, -0.35], [0, -1, 0, 0],[0,0,0,1]])
    print("aTg: \n", aTg)

    alfa = np.radians(10) #stupnjevi, kut otvaranja
    aTa_ = np.array([[np.cos(alfa), np.sin(alfa), 0, 0], [-np.sin(alfa), np.cos(alfa), 0, 0],
                    [0, 0, 1, 0], [0, 0 ,0 ,1]])
    print("aTa': \n", aTa_)

    bTt_ = np.linalg.multi_dot([bTt, tTg, np.linalg.inv(aTg), aTa_, aTg, np.linalg.inv(tTg)])
    print("umnozak rjesenje (bTt'): \n", bTt_)

    prvi = np.matmul(bTt, tTg)
    drugi = np.matmul(prvi,np.linalg.inv(aTg))
    treci = np.matmul(drugi, aTa_)
    cet = np.matmul(treci, aTg)
    pet = np.matmul(cet,np.linalg.inv(tTg) )
    print("rj: \n", pet)

    bRt_ = np.array(bTt_[0:3,0:3])
    eulerAngles = euler_from_matrix(bRt_)
    print("Rx, Ry, Rz: ", eulerAngles)
    qx,qy,qz,qw = quaternion_from_euler(eulerAngles[0], eulerAngles[1],eulerAngles[2])
    qx1,qy1,qz1,qw1 = quaternion_from_matrix(bTt_)

    print("\n KVAT (euler): ", qx,qy,qz,qw )
    print("\n KVAT (matrix): ", qx1,qy1,qz1,qw1 )

    firstPoint = poz.copy()
    firstPoint[0] = -1*(poz[0] + bTt_[0,3])
    firstPoint[1] = -1*(poz[1] + bTt_[1,3])
    print(firstPoint[0], firstPoint[1])
    print("------", bTt_[0,3])

    goal_joints = None
    while(goal_joints is None):
        goal_joints = RC.ik.get_ik(joints, firstPoint[0], firstPoint[1], firstPoint[2], qx, qy, qz, qw)
 
    goal_joints = list(goal_joints)
    print("Trenutni joints: \n", joints)
    print("Zeljeni joints: \n",goal_joints)

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

    print("Trenutni poz: \n", poz)
    print("Zeljeni poz: \n", firstPoint)

    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("spojen prva tocka okreta, ", s)
    #MORA SE PONISTIT OKRETANJE KORDINARNOG SUSTAVA IZ RVIZA , JER SE OVDJE SALJE STVARNI KOORDINATNI SUSTAV ROBOTA
    firstPoint[0] *= -1
    firstPoint[1] *= -1

    #firstPoint = [-0.4050754502702911, -0.5859013635888133, 0.3860864483389306, 1.491725232716374, -0.6391006558116826, 0.6156069651657792]
    #goal_joints = [4.323294828347154, -2.6539417124412386, -0.47583403536390456, 0.009209058171513122, -1.015597183358867, 3.932554911750079]
    s.sendall(("movej(get_inverse_kin(p"+ str(firstPoint) +", qnear="+ str(goal_joints)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
    time.sleep(2)
    print("End prva tocka okreta")
    data=s.recv(1024)
    s.close()
    time.sleep(2)

    print("Trenutni poz: \n", poz)
    print("Zeljeni poz: \n", firstPoint)


    


#     # instanca upravljaca robotom

#     RC = RobotControl()
#     # plan=RC.setPosition(-0.4, -0.4, 0.8)
#     # RC.executeTrajectory(plan)

#     #current_joints = RC.move_group.get_current_joint_values() #move joint vrijednosti iz polyscopa
#     #print(current_joints)


#     HOST = "192.168.22.14"
#     PORT = 30002

#     #postavljanje robota u pocetnu poziciju
#     s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#     s.connect((HOST,PORT))
#     print("spojen start, ", s)
#     start_tocka = [-0.3699745933297053, -0.5186108669590446, 0.3980670206407025, 1.4574027968490066, -0.5868335536521441, 0.6285672381695145]
#     start_joints = [4.2015700340271, -1.9716318289386194, -1.1473305861102503, 3.1599974632263184, -2.0953381697284144, 0.8027371168136597]
#     s.sendall(("movej(get_inverse_kin(p"+ str(start_tocka) +", qnear="+ str(start_joints)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
#     print("End start poz")
#     data=s.recv(1024)
#     s.close()

#     RC.openGripper()
#     time.sleep(10)


#     #prva transformacija za 22.5 stupnjeva
#     poz, joints = RC.getTCPvalues()
#     #print("tip poz: ", type(poz))
#     print("star_poz: ", poz)

#     novi_poz = RC.calculateNextStep(poz)  #tu zelim da se pomakne
#     #print("novi_poz: ", novi_poz)

#     quaternion = quaternion_from_euler(novi_poz[3], novi_poz[4], novi_poz[5])
#     qx,qy,qz,qw = quaternion

#     goal_joints = RC.ik.get_ik(joints, novi_poz[0], novi_poz[1], novi_poz[2], qx, qy, qz, qw)

#     print("Stari: ", joints)
#     print("GOAL:", goal_joints)
#     goal_joints = list(goal_joints)

#     #print("GOAL:", goal_joints)

#     for i in range(0,len(goal_joints)):
#         print(np.degrees(goal_joints[i]))

#     # joint_goal = RC.move_group.get_current_joint_values()
#     # joint_goal[0] = goal_joints[0]
#     # joint_goal[1] = goal_joints[1]
#     # joint_goal[2] = goal_joints[2]
#     # joint_goal[3] = goal_joints[3]
#     # joint_goal[4] = goal_joints[4]
#     # joint_goal[5] = goal_joints[5]

#     # RC.move_group.set_joint_value_target(joint_goal)
#     # plan = RC.move_group.plan()

#     # RC.move_group.stop()

#     s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#     s.connect((HOST,PORT))
#     print("spojen prva tocka okreta, ", s)
#     #MORA SE PONISTIT OKRETANJE KORDINARNOG SUSTAVA IZ RVIZA , JER SE OVDJE SALJE STVARNI KOORDINATNI SUSTAV ROBOTA
#     novi_poz[0] *= -1
#     novi_poz[1] *= -1
#     s.sendall(("movej(get_inverse_kin(p"+ str(novi_poz) +", qnear="+ str(goal_joints)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
#     print("End prva tocka okreta")
#     data=s.recv(1024)
#     s.close()
#     time.sleep(10)


#     ##DRUGA TOČKA
#     novi_poz2 = RC.calculateNextStep(novi_poz)
#     print("\n novi_poz BROJ 2: ", novi_poz2)

#     quaternion2 = quaternion_from_euler(novi_poz2[3], novi_poz2[4], novi_poz2[5])
#     qx,qy,qz,qw = quaternion2

#     goal_joints2 = RC.ik.get_ik(goal_joints, novi_poz2[0], novi_poz2[1], novi_poz2[2], qx, qy, qz, qw)
#     print("!!! GOAL:", goal_joints2)
#     goal_joints2 = list(goal_joints2)

#     print("GOAL:", goal_joints2)

#     # joint_goal = RC.move_group.get_current_joint_values()
#     # joint_goal[0] = goal_joints2[0]
#     # joint_goal[1] = goal_joints2[1]
#     # joint_goal[2] = goal_joints2[2]
#     # joint_goal[3] = goal_joints2[3]
#     # joint_goal[4] = goal_joints2[4]
#     # joint_goal[5] = goal_joints2[5]

#     # RC.move_group.set_joint_value_target(joint_goal)
#     # plan = RC.move_group.plan()

#     # RC.move_group.stop()

#     s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#     s.connect((HOST,PORT))
#     print("spojen tocka 1, ", s)

#     #MORA SE PONISTIT OKRETANJE KORDINARNOG SUSTAVA IZ RVIZA , JER SE OVDJE SALJE STVARNI KOORDINATNI SUSTAV ROBOTA
#     novi_poz2[0] *= -1
#     novi_poz2[1] *= -1
  
#     s.sendall(("movej(get_inverse_kin(p"+ str(novi_poz2) +", qnear="+ str(goal_joints2)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
 
#     print("End tocka2")
#     data=s.recv(1024)
#     s.close()
#     time.sleep(6)


# ##TREECA TOČKA
#     novi_poz3 = RC.calculateNextStep(novi_poz2)
#     print("\n novi_poz BROJ 3: ", novi_poz3)

#     quaternion3 = quaternion_from_euler(novi_poz3[3], novi_poz3[4], novi_poz3[5])
#     qx,qy,qz,qw = quaternion3

#     goal_joints3 = RC.ik.get_ik(goal_joints2, novi_poz3[0], novi_poz3[1], novi_poz3[2], qx, qy, qz, qw)
#     print("!!! GOAL:", goal_joints3)
#     goal_joints2 = list(goal_joints3)

#     print("GOAL:", goal_joints3)

#     # joint_goal = RC.move_group.get_current_joint_values()
#     # joint_goal[0] = goal_joints2[0]
#     # joint_goal[1] = goal_joints2[1]
#     # joint_goal[2] = goal_joints2[2]
#     # joint_goal[3] = goal_joints2[3]
#     # joint_goal[4] = goal_joints2[4]
#     # joint_goal[5] = goal_joints2[5]

#     # RC.move_group.set_joint_value_target(joint_goal)
#     # plan = RC.move_group.plan()

#     # RC.move_group.stop()

#     s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#     s.connect((HOST,PORT))
#     print("spojen tocka 3, ", s)

#     #MORA SE PONISTIT OKRETANJE KORDINARNOG SUSTAVA IZ RVIZA , JER SE OVDJE SALJE STVARNI KOORDINATNI SUSTAV ROBOTA
#     novi_poz3[0] *= -1
#     novi_poz3[1] *= -1
  
#     s.sendall(("movej(get_inverse_kin(p"+ str(novi_poz3) +", qnear="+ str(goal_joints3)+"),a=1.4,v=0.2)"+"\n").encode('utf-8'))
 
#     print("End tocka3")
#     data=s.recv(1024)
#     s.close()




#     # Tocka 1
#     #RC.setPosition(-0.4, -0.40, 0.8)
#     #RC.executeTrajectory(plan)
#     print("Finished point 1.")

#     # # Otvori gripper
#     #RC.openGripper()

#     # Zatvori gripper
#     #RC.closeGripper()

#     # Tocka 2
#     # plan = RC.setPosition(-0.3, -0.30, 0.8)
#     # RC.executeTrajectory(plan)
#     # print("Finished point 2.")


#     # Otvori gripper
#     #RC.openGripper()

#     print("Sequence done!")

#     rospy.spin()
 

# #[-0.1714525672191185, -0.5752180357905977, 0.3990368460928424, 1.450011982914319, -0.5325162737000843, 0.5892626994523873] #---> mm, rad
# #[4.588145732879639, -1.8396227995501917, -1.3286092917071741, 3.2249526977539062, -1.7115009466754358, 0.7316369414329529] #--->joints
