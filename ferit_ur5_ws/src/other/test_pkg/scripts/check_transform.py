import rospy
import tf2_ros
import geometry_msgs.msg

rospy.init_node('tf_listener')
tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

try:
    trans = tf_buffer.lookup_transform('base_link', 'tool0', rospy.Time(0), rospy.Duration(1.0))
    print(trans)
except Exception as e:
    print("Transform not available:", e)