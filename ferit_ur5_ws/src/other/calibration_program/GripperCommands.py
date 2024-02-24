import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

class GripperCommands:
    def __init__(self):
        #gripper init
        self.gripper_pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=5)
        
        self.gripper = Robotiq3FGripperRobotOutput()
        self.gripper.rACT = 1
        self.gripper.rMOD = 1
        self.gripper.rGTO = 1
        self.gripper.rATR = 0
        self.gripper.rPRA = 255
        self.gripper.rSPA = 5
        self.gripper.rFRA = 150
        
        
    def eval(self, input):
        input_split = input.split(' ', 1)
        command = input_split[0]
        arguments = ""
        if(len(input_split) == 2):
            arguments = input_split[1]
        
        if(command == ""):
            self.printHelp()
        else:
            if(command == "grip"):
                self.command_gripper_grip()
            elif(command == "release"):
                self.command_gripper_release()
            else:
                self.printHelp()

    def printHelp(self):
        print("gripper command is used to deal with gripper")
        print("gripper grip - closes gripper on robot")
        print("gripper release - opens gripper on robot")
        print("gripper help - shows this message")
        return 0;

    #napravljeno prema primjeru
    #https://docs.ros.org/en/kinetic/api/robotiq_3f_gripper_control/html/Robotiq3FGripperSimpleController_8py_source.html

    def command_gripper_grip(self):
        self.gripper.rPRA = 255
        self.gripper_pub.publish(self.gripper);
        return

    def command_gripper_release(self):
        self.gripper.rPRA = 0 
        self.gripper_pub.publish(self.gripper); 
        return
