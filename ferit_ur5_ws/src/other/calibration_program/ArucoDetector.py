import os
import cv2 
# import aruco
import cv2.aruco as aruco

class ArucoDetector:
  def __init__(self, camera_params_path:str, aruco_dict_path: str):

    fs = cv2.FileStorage(camera_params_path, cv2.FILE_STORAGE_READ)
    camera_mat = fs.getNode('camera_matrix').mat()
    dist_coeffs = fs.getNode('distortion_coefficients').mat()



    # self.camparam = aruco.CameraParameters(camera_mat, dist_coeffs, (480, 640))
    self.camparam = aruco.CameraParameters()
    # paramPath = os.path.join(os.path.dirname(__file__), "kamera-parameters.yml")
    # paramPath = os.path.join(os.path.dirname(__file__), "camera_parameters.yml")
    # dictPath = os.path.join(os.path.dirname(__file__), "4x4_1000.dict")
    paramPath = os.path.join(camera_params_path)
    dictPath = os.path.join(aruco_dict_path)   

    print(self.camparam.isValid())

    self.camparam.readFromXMLFile('/camera_parameters_asus.yml')
    # create detector and get parameters
    self.detector = aruco.MarkerDetector()
    self.detector.setDictionary(dictPath)
    #self.params = self.detector.getParameters()
