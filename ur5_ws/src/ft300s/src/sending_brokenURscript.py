#!/usr/bin/env python3
import socket
import time
import moveit_commander 
import roslib;

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

HOST="192.168.22.14"
PORT=30002


rospy.init_node('Robotiq3FGripperSimpleController')
pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)



def openGripper():
    command = Robotiq3FGripperRobotOutput();
    command.rACT = 1
    command.rGTO = 1
    command.rSPA = 255
    command.rFRA = 150
    command.rATR = 0
    command.rMOD = 1
    command.rPRA = 0
    start_time = time.time()
        
    while True:
        pub.publish(command)
        rospy.sleep(0.1)

        end_time = time.time()

        if float(end_time - start_time) >= 3.0:
            print("zavrsioooooooooooo")
            break
    

def closeGripper():
    command = Robotiq3FGripperRobotOutput();
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
        pub.publish(command)
        rospy.sleep(0.1)

        end_time = time.time()

        if float(end_time - start_time) >= 3.0:
            break



if __name__ == '__main__':
    # openGripper()
    # time.sleep(0.5)

    # print("Starting program")

    # s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    # s.connect((HOST,PORT))
    # time.sleep(1)
    # f = open("/home/lukrecia/catkin_ws/src/ft300s/src/nova.txt", "rb")
    # l = f.read()
    # s.sendall(l)
    # time.sleep(5)
    # print("End program")
    # data=s.recv(1024)
    # s.close()

    # time.sleep(0.5)

    # s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    # s.connect((HOST,PORT))
    # time.sleep(1)
    # f = open("/home/lukrecia/catkin_ws/src/ft300s/src/poc_poz.txt", "rb")
    # l = f.read()
    # s.sendall(l)
    # time.sleep(5)
    # print("End program")
    # data=s.recv(1024)
    # s.close()

    # time.sleep(0.5)

    # s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    # s.connect((HOST,PORT))
    # time.sleep(1)
    # s.sendall(("zero_ftsensor()" + "\n").encode('utf-8')) 
    # time.sleep(2)
    # s.close()

    # time.sleep(0.5)

    # closeGripper()

    # time.sleep(2)
    # s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    # s.connect((HOST,PORT))
    # time.sleep(1)
    # f = open("/home/lukrecia/catkin_ws/src/ft300s/src/otvaranje_ladice.txt", "rb")
    # l = f.read()
    # print(l)

    # s.sendall(l)
    # time.sleep(5)

    # print("End program")
    # data=s.recv(1024)
    # s.close()



    # time.sleep(2)

    # s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    # s.connect((HOST,PORT))
    # time.sleep(1)
    # f = open("/home/lukrecia/catkin_ws/src/ft300s/src/zatvaranje_ladice.txt", "rb")
    # l = f.read()
    # print(l)

    # s.sendall(l)
    # time.sleep(5)

    # print("End program")
    # data=s.recv(1024)
    # s.close()

    # print("Receved",repr(data))

    # time.sleep(0.5)

    # openGripper()

    # time.sleep(1)

    # print("sad cu se spojit")
    # s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    # s.connect((HOST,PORT))
    # print("spojen, ", s)
  
    # f = open("/home/lukrecia/catkin_ws/src/ft300s/src/zadnja_poz.txt", "rb")
    # l = f.read()
    # s.sendall(l)
 
    # print("End program")
    # data=s.recv(1024)
    # s.close()

    # print("Receved",repr(data))
    

    time.sleep(1)

    print("sad cu se spojit")
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))
    print("spojen, ", s)
  
    f = open("/home/lukrecia/catkin_ws/src/ft300s/src/ona.txt", "rb")
    l = f.read()
    s.sendall(l)
 
    print("End program")
    data=s.recv(1024)
    s.close()

    print("Receved",repr(data))
