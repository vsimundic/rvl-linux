import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import json
import pose_matrix_conversion

class MarkerCommands:
    def __init__(self, robotComms):
        self.markers = {} #marker positions
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
            if(command == "list"):
                self.command_list()
            elif(command == "rm"):
                self.command_rm(arguments)
            elif (command == "capture"):
                self.command_capture(arguments)
            elif(command == "save"):
                self.command_save(arguments)
            elif(command == "load"):
                self.command_load(arguments)
            elif(command == "saveall"):
                self.command_saveAll(arguments)
            elif(command == "help"):
                self.printHelp()
            else:
                self.printHelp()

    def printHelp(self):
        print("marker command is used to work with list of markers")
        print("marker subcomands are:")
        print("marker list - lists captured positions")
        print("marker rm <id> - removes marker identified by id from list")
        print("marker capture <id> - captures marker to program memory")
        print("marker save <id>  <path> - saves marker to path")
        print("marker load <id> <path> - loads marker from path")
        print("marker saveall <folder> - saves all markers to specified folder")
        print("marker help - prints this message")
        return 0;
        
    def command_list(self):
        for key in self.markers.keys():
            marker = self.markers[key]
            print(type(marker))
            print("Marker {}".format(key))
            print(marker)
            print("")
        return

    def command_rm(self, arguments):
        key = int(arguments)
        if key in self.markers:
            del self.markers[key]
            print("Removed marker {}".format(key))
        else:
            print("Unknown key {}".format(key))
        return

    def command_capture(self, arguments):
        key = int(arguments)
        pose = self.robotComms.pose()
        self.markers[key] = dict()
        self.markers[key]["pose"] = pose
        self.markers[key]["position"] = pose_matrix_conversion.pose_to_matrix(pose)
        return

    def command_save(self, arguments):
        [id, path] =  arguments.split(' ', 1)
        key = int(id)
        if key in self.markers.keys():
            with open(path, "w") as f:
                data = dict()
                data["position"] = dict()
                data["orientation"] = dict()
                data["position"]["x"] = self.markers[key]["pose"].position.x
                data["position"]["y"] = self.markers[key]["pose"].position.y
                data["position"]["z"] = self.markers[key]["pose"].position.z
                data["orientation"]["x"] = self.markers[key]["pose"].orientation.x
                data["orientation"]["y"] = self.markers[key]["pose"].orientation.y
                data["orientation"]["z"] = self.markers[key]["pose"].orientation.z
                data["orientation"]["w"] = self.markers[key]["pose"].orientation.w
                json.dump(data, f)
            return

    def command_load(self, arguments):
        [id, path] =  arguments.split(' ', 1)
        key = int(id)
        with open(path, "r") as f:
            data = json.load(f)
            pose = geometry_msgs.msg.Pose()
            pose.position.x = data["position"]["x"]
            pose.position.y = data["position"]["y"]
            pose.position.z = data["position"]["z"]
            pose.orientation.x = data["orientation"]["x"]
            pose.orientation.y = data["orientation"]["y"]
            pose.orientation.z = data["orientation"]["z"]
            pose.orientation.w = data["orientation"]["w"]
            self.markers[key] = dict()
            self.markers[key]["pose"] = pose
            self.markers[key]["position"] = pose_matrix_conversion.pose_to_matrix(pose)
            print("Loaded marker {} from file:".format(id))
            print(pose)
            return pose

    def command_saveAll(self, arguments):
        path = arguments
        for key in self.markers.keys():
            key_path = path+"/marker"+str(key)+".json"
            self.command_save(str(key)+" "+key_path)
        return
