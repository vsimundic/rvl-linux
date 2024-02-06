import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

class CameraReader:
  lastimage = False
  imagesub = False
  bridge = CvBridge()
  
  def __init__(self, image_topic: str):
    self.imagesub = rospy.Subscriber(image_topic, Image, self.image_callback)
    # self.imagesub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

  def image_callback(self, camera_image):
    try:
      cv2_image = self.bridge.imgmsg_to_cv2(camera_image, "bgr8")
    except CvBridgeError as e:
      print(e)
    else:
      self.lastimage = cv2_image
