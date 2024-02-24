import pose_matrix_conversion

class RobotCommands:
    def __init__(self, robotComms):
        self.robotComms = robotComms

    def eval(self, input):
        input_split = input.split(' ', 1)
        command = input_split[0]
        arguments = ""
        if(len(input_split) == 2):
            arguments = input_split[1]
        
        if(command == ""):
            self.printHelp()
        else:
            if(command == "moveTo"):
                self.command_moveTo(arguments)
            elif(command == "pose"):
                self.command_pose()
            elif (command == "help"):
               self.printHelp()
            else:
                self.printHelp()

    def printHelp(self):
        print("robot command is used to deal with robot features")
        print("robot subcomands are:")
        print("robot moveTo <px> <py> <pz> <ox> <oy> <oz> <ow> - takes position parameters that robot should be in and moves robotic hand to that position")
        print("robot pose - prints current position of the robot")
        print("robot help - shows this message")
        return 0;
        
    def command_moveTo(self, arguments):

        split = arguments.split(' ', 7)

        if(len(split) < 7):
            return
        
        px = float(split[0])
        py = float(split[1])
        pz = float(split[2])
        ox = float(split[3])
        oy = float(split[4])
        oz = float(split[5])
        ow = float(split[6])
        
        self.robotComms.moveTo(px, py, pz, ox, oy, oz, ow)
        return
        
    def command_pose(self):
        current_pose = self.robotComms.pose()
        print(current_pose)
        print("\n")
        current_matrix = pose_matrix_conversion.pose_to_matrix(current_pose)
        print(current_matrix)
        
        return [current_pose, current_matrix]
