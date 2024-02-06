import os
import cv2
import aruco
import numpy as np
import pose_matrix_conversion

class CameraCommands:
  def __init__(self, cameraReader, ArUco, robotComms):
    self.images = []
    self.ArUco = ArUco
    self.cameraReader = cameraReader
    self.robotComms = robotComms
    self.E_T_C = None
    self.tool_E = None
    
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
      elif(command == "show"):
        self.command_show(arguments)
      elif(command == "aruco"):
        self.command_showaruco(arguments)
      elif(command == "info"):
        self.command_info(arguments)
      elif(command == "load"):
        self.command_load(arguments)
      elif(command == "save"):
        self.command_save(arguments)
      elif(command == "coords"):
        self.command_marker_coords(arguments)
      else:
        self.printHelp()

  def printHelp(self):
    print("camera command is used to deal with camera")
    print("camera subcomands are:")
    print("camera show - shows current image from camera")
    print("camera aruco - shows current image with aruco merker detection overlay")
    print("camera help - shows this message")
    print("camera coords <marker index> - calculates coordinates to center of marker from current camera image")
    return 0;

  def command_list(self):
      for index in range(0, len(self.images)):
          image = self.images[index]
          print("{}: {}".format(index, image["pose"]))

  def command_show(self, arguments):
    image = self.cameraReader.lastimage.copy()
    if 'DISPLAY' in os.environ.keys():
      cv2.imshow("image", image)
      cv2.waitKey(0)
    else:
      print("No display!")
    
  def command_showaruco(self, arguments):
    image = self.cameraReader.lastimage.copy()
    markers = self.ArUco.detector.detect(image, self.ArUco.camparam, 0.15)
    for marker in markers:
      print("Marker: {:d}".format(marker.id))
      print("center: {}".format(marker.getCenter()))
      mtx = marker.getTransformMatrix()
      print("M: {}".format(np.array(mtx)))
      marker.draw(image, np.array([255, 255, 255]), 2)
      marker.calculateExtrinsics(0.15, self.ArUco.camparam)
      aruco.CvDrawingUtils.draw3dAxis(image, self.ArUco.camparam, marker.Rvec, marker.Tvec, .1)
    if 'DISPLAY' in os.environ.keys():
      cv2.imshow("image", image)
      cv2.waitKey(0)
    else:
      print("No display!")
      
  def command_info(self, arguments):
    index = int(arguments)
    image = self.images[index]
    markers = self.ArUco.detector.detect(image["image"], self.ArUco.camparam, 0.15)
    print("Image {}".format(index))
    print("Position: {}".format(image["pose"]))
    print("\nDetected markers:")
    for marker in markers:
      print("marker {}, center: {}".format(marker.id, marker.getCenter()))

  def command_load(self, arugments):
    [pose, path] = arugments.split(' ', 1)
    image = cv2.imread(path)
    image = {"pose": pose, "image": image}
    self.images.append(image)
    
  def command_save(self, arguments):
    [index, path] = arguments.split(' ', 1)
    image = self.images[int(index)]
    cv2.imwrite(path, image["image"])
    with open(path+".pose", "w") as f:
      f.write(str(image["pose"]))


  def command_marker_coords(self, arguments):
    if(self.E_T_C is None):
      print("calibration needs to be done before marker center could be calculated")
      return
    markerIndex = arguments
    markerIndex = int(markerIndex)

    image = self.cameraReader.lastimage.copy()
    markers = self.ArUco.detector.detect(image, self.ArUco.camparam, 0.15)

    marker_matrix = None
    
    for marker in markers:
      if(marker.id == markerIndex):
        marker_matrix = marker.getTransformMatrix()
        break
    
    if(marker_matrix is None):
      print("Marker not detected.")
      return

    pose = self.robotComms.pose()
    position = pose_matrix_conversion.pose_to_matrix(pose)
    
    c_p_a = marker_matrix[:,3]
    B_T_E = position
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
