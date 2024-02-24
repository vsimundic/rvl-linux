import os
import cv2
import aruco
import numpy as np
import pose_matrix_conversion
import json
import geometry_msgs.msg
import yaml
import matplotlib.pyplot as plt

class ImageCommands:
  def __init__(self, cameraReader, ArUco, robotComms):
    self.images = []
    self.ArUco = ArUco
    self.cameraReader = cameraReader
    self.robotComms = robotComms
    self.E_T_C = None
    self.tool_E = None
    self.markers = None
    
  def eval(self, input):
    input_split = input.split(' ', 1)
    command = input_split[0]
    arguments = ""
    if(len(input_split) == 2):
      arguments = input_split[1]
      
    if(command == ""):
      self.command_help()
    else:
      if(command == "list"):
        self.command_list()
      elif(command == "rm"):
        self.command_remove(arguments)
      elif(command == "show"):
        self.command_show(arguments)
      elif(command == "aruco"):
        self.command_showaruco(arguments)
      elif(command == "capture"):
        self.command_capture()
      elif(command == "info"):
        self.command_info(arguments)
      elif(command == "load"):
        self.command_load(arguments)
      elif(command == "save"):
        self.command_save(arguments)
      elif(command == "saveall"):
        self.command_saveAll(arguments)
      elif(command == "visualize"):
        self.command_showvisualization(arguments)
      elif(command == "coords"):
        self.command_marker_coords(arguments)
      elif(command == "help"):
        self.command_help()
      else:
        self.command_help()

  def command_help(self):
    print("image command is used to capture images for calibration calculation")
    print("image subcomands are:")
    print("image list - lists images currently in list")
    print("image rm <index> - removes image from list")
    print("image show <index> - shows image from list using cv2")
    print("image aruco <index> - shows image from list and marks detected aruco markers")
    print("image capture - takes image and saves it in list")
    print("image info <index> - shows information about image")
    print("image load <pose> <path> - loads image from path into list")
    print("image save <index> <path> - saves image to path")
    print("image saveall <index> <path> - saves images to path")
    print("image visualize <index> - shows visualization for image")
    print("image coords <image index> <marker index> - calculates coordinates to center of marker from image")
    print("image help - shows this text")
    return 0;

  def command_list(self):
      for index in range(0, len(self.images)):
          image = self.images[index]
          print("image {}:\n {}\n".format(index, image["pose"]))

  def command_remove(self, arguments):
    index = int(arguments)
    del self.images[index]
          
  def command_show(self, arguments):
    index = int(arguments)
    image = self.images[index]["image"]
    if 'DISPLAY' in os.environ.keys():
      cv2.imshow("image", image)
      cv2.waitKey(0)
    else:
      print("No display!")

  def command_capture(self):
    image = self.cameraReader.lastimage.copy()
    pose = self.robotComms.pose()
    markers = self.ArUco.detector.detect(image, self.ArUco.camparam, 0.15)
    marker_list = []
    for marker in markers:
      marker_matrix = marker.getTransformMatrix()
      marker_list.append({"id": marker.id, "matrix": marker_matrix, "center":marker.getCenter(), "position":np.array([marker_matrix[0][3], marker_matrix[1][3], marker_matrix[2][3], 1])}) #TODO check position
    self.images.append({"position": pose_matrix_conversion.pose_to_matrix(pose), "pose": pose, "image": image, "markers": marker_list})
    return
    
  def command_showaruco(self, arguments):
    index = int(arguments)
    image = self.images[index]["image"].copy()
    markers = self.ArUco.detector.detect(image, self.ArUco.camparam, 0.15)
    for marker in markers:
      print("Marker: {:d}".format(marker.id))
      print("center: {}".format(marker.getCenter()))
      mtx = marker.getTransformMatrix()
      print("M: {}".format(mtx))
      marker.draw(image, np.array([255, 255, 255]), 2)
      marker.calculateExtrinsics(0.15, self.ArUco.camparam)
      aruco.CvDrawingUtils.draw3dAxis(image, self.ArUco.camparam, marker.Rvec, marker.Tvec, .1)
    if 'DISPLAY' in os.environ.keys():
      cv2.imshow("image", image)
      cv2.waitKey(0)
    else:
      print("No display!")

  def command_showvisualization(self, arguments):
    if(self.E_T_C is None):
      print("calibration needs to be done before centers could be visualized")
      return
    
    index = int(arguments)
    image = self.images[index]
    imgplot = plt.imshow(image["image"])

    K = np.array([[597.9033203125, 0.0, 323.8436584472656], [0.0, 598.47998046875, 236.32774353027344], [0.0, 0.0, 1.0]])
    B_T_C = image["position"] @ self.E_T_C
    C_T_B = np.linalg.inv(B_T_C)
    for marker in image["markers"]:            
        marker_aruco_imgpos = K @ np.expand_dims(marker["position"][:3],1)
        marker_aruco_imgpos = marker_aruco_imgpos[:2,0] / marker_aruco_imgpos[2]
        plt.plot(marker_aruco_imgpos[0], marker_aruco_imgpos[1], 'b+')
        marker_robot_B = self.markers[marker["id"]]["position"] @ self.tool_E
        marker_robot_C = C_T_B @ marker_robot_B
        marker_robot_imgpos = K @ np.expand_dims(marker_robot_C[:3],1)
        marker_robot_imgpos = marker_robot_imgpos[:2,0] / marker_robot_imgpos[2]
        plt.plot(marker_robot_imgpos[0], marker_robot_imgpos[1], 'g+')
    plt.show()
  
  def command_info(self, arguments):
    index = int(arguments)
    image = self.images[index]
    print("Image {}".format(index))
    print("Pose: {}".format(image["pose"]))
    print("Position: {}".format(image["position"]))
    print("\nDetected markers:")
    for marker in image["markers"]:
      print("marker {}, center: {}".format(marker["id"], marker["center"]))

  def command_load(self, arugments):
    path = arugments
    image_data = cv2.imread(path)
    image = {"image": image_data}
    with open(path+".pose", "r") as f:
      data = json.load(f)
      #data = yaml.safe_load(f)
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

    markers = self.ArUco.detector.detect(image_data, self.ArUco.camparam, 0.15)
    marker_list = []
    for marker in markers:
      marker_matrix = marker.getTransformMatrix()
      marker_list.append({"id": marker.id, "matrix": marker_matrix, "center":marker.getCenter(), "position":np.array([marker_matrix[0][3], marker_matrix[1][3], marker_matrix[2][3], 1])})
    image["markers"] = marker_list
    self.images.append(image)
    
  def command_save(self, arguments):
    [index, path] = arguments.split(' ', 1)
    index = int(index)
    image = self.images[index]
    cv2.imwrite(path, image["image"])
    with open(path+".pose", "w") as f:
      data = dict()
      data["position"] = dict()
      data["orientation"] = dict()
      data["position"]["x"] = self.images[index]["pose"].position.x
      data["position"]["y"] = self.images[index]["pose"].position.y
      data["position"]["z"] = self.images[index]["pose"].position.z
      data["orientation"]["x"] = self.images[index]["pose"].orientation.x
      data["orientation"]["y"] = self.images[index]["pose"].orientation.y
      data["orientation"]["z"] = self.images[index]["pose"].orientation.z
      data["orientation"]["w"] = self.images[index]["pose"].orientation.w
      json.dump(data, f)


  def command_saveAll(self, arguments):
    [path] = arguments.split(' ', 1)
    images_len = len(self.images)
    for i in range(0, images_len):
      img_path = path+"/image"+str(i)+".jpg"
      self.command_save(str(i)+" "+img_path)
    return

  def command_marker_coords(self, arguments):
    if(self.E_T_C is None):
      print("calibration needs to be done before centers could be calculated")
      return
    [imageIndex, markerIndex] = arguments.split(' ', 1)
    imageIndex = int(imageIndex)
    markerIndex = int(markerIndex)

    mId = -1
    for i,m in enumerate(self.images[imageIndex]["markers"]):
      if m["id"] == markerIndex:
        mId = i
        break

    if(mId == -1):
      print("Marker not detected.")
      return
      
    c_p_a = self.images[imageIndex]["markers"][mId]["matrix"][:,3]
    B_T_E = self.images[imageIndex]["position"]
    B_R_Ec = np.eye(4)
    B_R_Ec[1,1] = -1
    B_R_Ec[2,2] = -1
    E_T_C = self.E_T_C
    E_p_A = self.tool_E
    c_p_a=np.expand_dims(c_p_a,1)
    E_p_A=np.expand_dims(E_p_A,1)
    
    B_t_Ec= B_T_E @ E_T_C @ c_p_a - B_R_Ec @ E_p_A
    delta=np.expand_dims(np.array([0, 0, 0.02, 0]), 1)
    B_t_Ec = B_t_Ec + delta
    print(B_t_Ec)
    print("with delta")
    print(delta)
