#!/usr/bin/env python

import os
import shutil
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rosnode import kill_nodes
from PIL import Image as PilImage
import RVLPYRGBD2PLY as rvl
import RVLPYDDDetector
import json 
from core.real_ur5_controller import UR5Controller

# Global counters and state
number_of_images = 0
key_counter = 0
last_save_time = 0
max_save_rate = 5.0  # Hz (overridable via param)

# Parameters (set in main)
base_dir = ""
rgb_dir = ""
depth_dir = ""
ply_dir = ""
model_output_path = ""
detector_config_path = ""
rgb_topic = ""
depth_topic = ""
node_name = "rgbAndDepth_image_subscriber"

def convert_numpy(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    else:
        return obj

def stop_node(node_name):
    kill_nodes([node_name])

def ensure_directories_exist(*dirs):
    for d in dirs:
        if not os.path.exists(d):
            os.makedirs(d)
            rospy.loginfo(f"Created directory: {d}")

def reset_directories(*dirs):
    """
    Delete everything inside the given directories (if they exist)
    and recreate the empty folders so each run starts from a clean slate.
    """
    for d in dirs:
        if os.path.exists(d):
            shutil.rmtree(d)
            rospy.loginfo(f"Cleared directory: {d}")
        os.makedirs(d, exist_ok=True)

def image_callback(rgb_msg, depth_msg):
    global number_of_images, key_counter, last_save_time
    global base_dir, rgb_dir, depth_dir, max_save_rate

    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough")

    cv2.imshow("RGB", rgb)
    key = cv2.waitKey(1)

    if key == 32:  # Space bar
        key_counter += 1

    if key_counter == 1:
        now = rospy.get_time()
        if now - last_save_time < 1.0 / max_save_rate:
            return  # Skip this frame

        last_save_time = now

        file_idx = str(number_of_images).zfill(4)
        rgb_path = os.path.join(base_dir, rgb_dir, f"{file_idx}.png")
        depth_path = os.path.join(base_dir, depth_dir, f"{file_idx}.png")

        cv2.imwrite(rgb_path, rgb)
        cv2.imwrite(depth_path, depth)
        number_of_images += 1
        rospy.loginfo(f"Saved image {file_idx}")

    if key_counter >= 2:
        cv2.destroyAllWindows()
        create_ply_images(number_of_images)
        stop_node(node_name)

def create_ply_images(count):
    global base_dir, rgb_dir, depth_dir, ply_dir

    rospy.loginfo("Creating PLY files...")
    rgbd2ply = rvl.RGBD2PLY()

    for i in range(count):
        file_idx = str(i).zfill(4)
        rgb_file = os.path.join(base_dir, rgb_dir, f"{file_idx}.png")
        depth_file = os.path.join(base_dir, depth_dir, f"{file_idx}.png")
        ply_file = os.path.join(base_dir, ply_dir, f"{file_idx}.ply")

        rgb_img = PilImage.open(rgb_file)
        rgb_data = np.array(rgb_img.getdata()).astype(np.byte)
        bgr_array = np.stack((rgb_data[:, 2], rgb_data[:, 1], rgb_data[:, 0]), axis=1)

        depth_img = PilImage.open(depth_file)
        depth_array = np.array(depth_img.getdata()).astype(np.short)

        rospy.loginfo(f"Converting to PLY: {file_idx}")
        rgbd2ply.pixel_array_to_ply(
            bgr_array,
            depth_array,
            597.9033203125, 598.47998046875,
            323.8436584472656, 236.32774353027344,
            640, 480,
            0.050,
            ply_file
        )

    rospy.loginfo("PLY generation complete.")

def build_model(count, robot):
    global base_dir, ply_dir, rgb_dir, model_output_path, detector_config_path

    rospy.loginfo("Starting 3D detection...")
    detector = RVLPYDDDetector.PYDDDetector()
    detector.create(detector_config_path)

    first_id = 0
    last_id = count - 1

    rospy.loginfo("Loading meshes:")
    for mesh_id in range(first_id, last_id):
        mesh_file = os.path.join(base_dir, ply_dir, f"{mesh_id:04d}.ply")
        rospy.loginfo(f"  {mesh_file}")
        detector.add_mesh(mesh_file)

    rospy.loginfo("Loading RGB images:")
    for img_id in range(first_id, last_id + 1):
        rgb_file = os.path.join(base_dir, rgb_dir, f"{img_id:04d}.png")
        rospy.loginfo(f"  {rgb_file}")
        detector.add_rgb(rgb_file)

    detector.set_hyp_file_name(os.path.join(base_dir, "hyps.txt"))
    ao_result = detector.detect()

    current_joints = robot.get_current_joint_values()

    ao_result["joint_values"] = current_joints

    # Convert to JSON-safe format
    ao_json = convert_numpy(ao_result)

    # Save as .json instead of .txt
    json_path = os.path.splitext(model_output_path)[0] + ".json"
    with open(json_path, "w") as f:
        json.dump(ao_json, f, indent=2)

    rospy.loginfo(f"Detection complete. Result saved to: {json_path}")
    return ao_result

def main():
    global base_dir, rgb_dir, depth_dir, ply_dir
    global model_output_path, detector_config_path
    global rgb_topic, depth_topic, max_save_rate

    rospy.init_node(node_name)

    # Load parameters
    base_dir = rospy.get_param("~base_dir", "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection")
    rgb_dir = rospy.get_param("~rgb_dir", "RGB_images")
    depth_dir = rospy.get_param("~depth_dir", "DEPTH_images")
    ply_dir = rospy.get_param("~ply_dir", "PLY_seg")
    model_output_path = rospy.get_param("~model_output_path", "models/doorModel.json")
    model_output_path = os.path.join(base_dir, model_output_path)
    detector_config_path = rospy.get_param("~detector_config_path", "/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg")
    rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
    depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
    max_save_rate = rospy.get_param("~save_fps", 5.0)

    # Build absolute paths for I/O folders
    rgb_path = os.path.join(base_dir, rgb_dir)
    depth_path = os.path.join(base_dir, depth_dir)
    ply_path = os.path.join(base_dir, ply_dir)

    reset_directories(rgb_path, depth_path, ply_path)

    # Ensure required directories exist
    ensure_directories_exist(
        rgb_path,
        depth_path,
        ply_path,
        os.path.dirname(model_output_path)
    )

    # Subscribers with synchronization
    rgb_sub = Subscriber(rgb_topic, Image)
    depth_sub = Subscriber(depth_topic, Image)
    sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
    sync.registerCallback(image_callback)

    rospy.loginfo("Ready. Press SPACE to start image capture.")
    rospy.spin()

    robot = UR5Controller()

    build_model(number_of_images, robot)

if __name__ == "__main__":
    main()