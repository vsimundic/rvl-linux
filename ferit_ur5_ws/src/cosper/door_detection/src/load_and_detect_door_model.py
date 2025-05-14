#!/usr/bin/env python
"""Detect a 3‑D model using a predefined list of frame names.

The list file is expected to contain one PNG file name per line and the word
"end" (case‑insensitive) marking the first logical break. Only the names that
appear *before* this marker are used.

Example list file
-----------------
0000.png
0001.png
0002.png
end

0014.png  # <-- ignored
0016.png  # <-- ignored

Run as a ROS node so that all paths and settings come from parameters, e.g.::

    rosrun my_pkg detect_from_list.py \
        _base_dir:=/path/to/exp \
        _rgb_dir:=RGB_images \
        _ply_dir:=PLY_seg \
        _detector_config_path:=/path/to/DDD2.cfg \
        _image_list_file:=frames.txt

Outputs a JSON file next to *model_output_path* (same name but *.json*).
"""

import os
import rospy
import json
import numpy as np
import RVLPYDDDetector
from core.real_ur5_controller import UR5Controller

# Parameters (filled in main)
base_dir = ""
rgb_dir = ""
ply_dir = ""
model_output_path = ""
detector_config_path = ""
image_list_file = ""

node_name = "model_detection_from_list"


def convert_numpy(obj):
    """Recursively convert numpy arrays so they can be saved as JSON."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [convert_numpy(i) for i in obj]
    return obj


def load_indices(list_path):
    """Return the integer indices (e.g. 0, 1, 2) that appear before the first 'end'."""
    indices = []
    with open(list_path, "r") as f:
        for line in f:
            token = line.strip()
            if not token:
                continue            # skip empty lines
            if token.lower() == "end":
                break               # stop at first 'end'
            if token.endswith(".png"):
                stem = os.path.splitext(token)[0]
                if stem.isdigit():
                    indices.append(int(stem))
                else:
                    rospy.logwarn(f"Skipping non‑numeric filename '{token}' in {list_path}")
            else:
                rospy.logwarn(f"Skipping unsupported line '{token}' in {list_path}")
    return sorted(indices)


def build_model(indices, robot):
    """Use the given frame *indices* to build meshes and perform detection."""
    global base_dir, ply_dir, rgb_dir, model_output_path, detector_config_path

    if not indices:
        rospy.logerr("No valid frame indices loaded; aborting.")
        return {}

    rospy.loginfo(f"Loaded {len(indices)} frame indices: {indices}")

    detector = RVLPYDDDetector.PYDDDetector()
    detector.create(detector_config_path)

    # Meshes: all but the last frame (aligning with the behaviour of the capture script)
    for idx in indices[:-1]:
        mesh_file = os.path.join(base_dir, ply_dir, f"{idx:04d}.ply")
        rospy.loginfo(f"Adding mesh {mesh_file}")
        detector.add_mesh(mesh_file)

    # RGB frames: *all* selected frames
    for idx in indices:
        rgb_file = os.path.join(base_dir, rgb_dir, f"{idx:04d}.png")
        rospy.loginfo(f"Adding RGB {rgb_file}")
        detector.add_rgb(rgb_file)

    detector.set_hyp_file_name(os.path.join(base_dir, "hyps.txt"))
    rospy.loginfo("Running detection …")
    ao_result = detector.detect()

    # Add current robot joint configuration for completeness
    ao_result["joint_values"] = robot.get_current_joint_values()

    # Convert numpy arrays→lists for JSON storage
    ao_json = convert_numpy(ao_result)

    json_path = os.path.splitext(model_output_path)[0] + ".json"
    with open(json_path, "w") as fp:
        json.dump(ao_json, fp, indent=2)
    rospy.loginfo(f"Detection finished; result written to {json_path}")

    return ao_result


def main():
    global base_dir, rgb_dir, ply_dir, model_output_path, detector_config_path, image_list_file

    rospy.init_node(node_name)

    # Parameters
    base_dir = rospy.get_param("~base_dir", "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250424/door_detection")
    rgb_dir = rospy.get_param("~rgb_dir", "RGB_images")
    ply_dir = rospy.get_param("~ply_dir", "PLY_seg")
    model_output_path = rospy.get_param("~model_output_path", "models/doorModel.json")
    model_output_path = os.path.join(base_dir, model_output_path)
    detector_config_path = rospy.get_param("~detector_config_path", "/home/RVLuser/rvl-linux/RVLRecognitionDemo_Cupec_DDD2_Detection.cfg")
    image_list_file = rospy.get_param("~image_list_file", "sceneSequence.txt")

    if not os.path.isabs(image_list_file):
        image_list_file = os.path.join(base_dir, image_list_file)

    if not os.path.exists(image_list_file):
        rospy.logfatal(f"Image list file '{image_list_file}' does not exist.")
        return

    indices = load_indices(image_list_file)
    if not indices:
        rospy.logfatal("No usable frames found; nothing to do.")
        return

    robot = UR5Controller()
    build_model(indices, robot)


if __name__ == "__main__":
    main()
