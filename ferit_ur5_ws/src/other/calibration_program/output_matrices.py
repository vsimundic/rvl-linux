import pose_matrix_conversion
import yaml
import cv2
import geometry_msgs.msg
import numpy as np
import json
import yaml
from ArucoDetector import ArucoDetector

ArUco = ArucoDetector()

imageDirPath = "/home/luka/Documents/syncthing/posao/praksa/images/"

outputPath = "/home/luka/Documents/syncthing/posao/praksa/izracun_test/data/"

for i in range(0, 63):
    path = imageDirPath+"image"+str(i)+".jpg"
    image_data = cv2.imread(path)
    image = {"image": image_data}
    with open(path+".pose", "r") as f:
      data = yaml.safe_load(f)
      print(data)
      with open(outputPath+"image"+str(i)+".jpg.pose", "w") as f2:
          json.dump(data, f2)
      pose = geometry_msgs.msg.Pose()
      pose.position.x = data["position"]["x"]
      pose.position.y = data["position"]["y"]
      pose.position.z = data["position"]["z"]
      pose.orientation.x = data["orientation"]["x"]
      pose.orientation.y = data["orientation"]["y"]
      pose.orientation.z = data["orientation"]["z"]
      pose.orientation.w = data["orientation"]["w"]
      image["pose"] = pose
      image["position"] = pose_matrix_conversion.pose_to_matrix(pose)
      markers = ArUco.detector.detect(image_data, ArUco.camparam, 0.15)
      marker_list = []
      for marker in markers:
          marker_matrix = marker.getTransformMatrix()
          marker_list.append({"id": marker.id, "matrix": marker_matrix.tolist()})
      image["markers"] = marker_list

    #output pose
    path=outputPath+"image"+str(i)+".jpg.pose"
    with open(path, "w") as f:
        data["position"]["x"] = image["pose"].position.x
        data["position"]["y"] = image["pose"].position.y
        data["position"]["z"] = image["pose"].position.z
        data["orientation"]["x"] = image["pose"].orientation.x
        data["orientation"]["y"] = image["pose"].orientation.y
        data["orientation"]["z"] = image["pose"].orientation.z
        data["orientation"]["w"] = image["pose"].orientation.w
        
        json.dump(data ,f, separators=(',', ':'), indent=4)

    #output matrix
    path=outputPath+"image"+str(i)+".jpg.matrix"
    with open(path, "w") as f:
        position=dict()
        position["position"] = image["position"].tolist()
        json.dump(position,f, separators=(',', ':'), indent=4)
        
    #output markers
    path=outputPath+"image"+str(i)+".jpg.markers.json"
    with open(path, "w") as f:
        markers = dict()
        markers["markers"] = image["markers"]
        json.dump(markers, f, separators=(',', ':'), indent=4)
          
markerDirPath = "/home/luka/Documents/syncthing/posao/praksa/markers/"
for i in range(1, 10):
    print("marker {}".format(i))
    path=markerDirPath+"marker"+str(i)+".json"
    print(path)
    marker = dict()
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
        marker = dict()
        marker["pose"] = pose
        marker["position"] = pose_matrix_conversion.pose_to_matrix(pose)
        
    #path=outputPath+"marker"+str(i)+".json"
    #with open(path, "w") as f:
    #    position=dict()
    #    position["position"] = marker["pose"]["position"].tolist()
    #    position["orientation"] = marker["pose"]["orientation"].tolist()
    #    json.dump(position,f, separators=(',', ':'), indent=4)
        
    path=outputPath+"marker"+str(i)+".matrix.json"
    with open(path, "w") as f:
        position=dict()
        position["position"] = marker["position"].tolist()
        json.dump(position,f, separators=(',', ':'), indent=4)
