#!/usr/bin/env python

from std_msgs.msg import Bool
from cv_bridge import CvBridge
import rospy
import os, sys
import numpy as np
import RVLPYDDDetector 
import rosnode
from generate_cabinet_urdf import generate_cabinet_urdf_from_door_panel
from gazebo_msgs.srv import SpawnModel, SetModelConfiguration, SetModelConfigurationRequest, DeleteModel
from geometry_msgs.msg import Pose, TransformStamped
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix
import tf
import rospkg
import json
from read_json import read_ao_dict_from_json
import tf2_ros
import moveit_commander

node_name = ''
cfg_path = ''
rgb_save_path = ''
depth_save_path = ''
ply_save_path = ''
package_path = ''
b_load_model = False
listener = ''
broadcaster = ''
pus_operation_pub = ''
TCT = ''
TTB = ''
TB0 = ''
scene = ''


def callback(b_build_model_msg):
    global node_name, cfg_path, rgb_save_path, depth_save_path, ply_save_path, package_path, b_load_model

    b_load_model = rospy.get_param('/load_model')

    if b_build_model_msg:
        ao = {}
        if not b_load_model:
            ao = detect_model(cfg_path, rgb_save_path, depth_save_path, ply_save_path)
            print(ao)
        else:
            ao = read_ao_dict_from_json(os.path.join(package_path, 'config', 'ao.txt'))

        TAC = np.eye(4)
        TAC[:3, :3] = ao['R'].copy()
        TAC[:3, 3] = ao['t'].copy()

        model_name = 'my_cabinet'

        # Delete model if exists
        print('Deleting and adding model.')
        delete_model(model_name)
        spawn_model(w_door=ao['s'][0], h_door=ao['s'][1], TAC=TAC, model_name=model_name)
        delete_model(model_name)
        spawn_model(w_door=ao['s'][0], h_door=ao['s'][1], TAC=TAC, model_name=model_name)

        
        # move_model_joint(model_name, 'joint_0', np.radians(-20))


        rospy.sleep(2)
        msg = Bool()
        msg.data = True
        pus_operation_pub.publish(msg)

        # rosnode.kill_nodes([node_name])

def delete_model(model_name):
    global listener
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        _ = spawn_urdf_model_proxy(model_name)
        rospy.loginfo('URDF deleted successfully')
    except rospy.ServiceException as e:
        rospy.logerr('Failed to delete model: %s' % e)


def spawn_model(w_door, h_door, TAC, model_name):
    model_xml, TXA = generate_cabinet_urdf_from_door_panel(w_door=w_door, 
                                                      h_door=h_door, 
                                                      d_door=0.018)

    Tz = np.eye(4)
    theta = np.radians(180)
    s = np.sin(theta)
    c = np.cos(theta)
    Tz[:3, :3] = np.array([[c, -s, 0.],
                            [s, c, 0.],
                            [0., 0., 1.]])
    
    # rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     try:
    #         trans, rot = listener.lookupTransform('door_link', 'base_cabinet_link', rospy.Time(0))
    #         break   
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue

    # TXA = quaternion_matrix(rot)
    # TXA[:3, 3] = trans.copy()

    # This is TX0 - centroid of the cabinet
    TX0 = TB0 @ TTB @ TCT @ TAC @ Tz @ TXA
    if TX0[2, 3] < (h_door/2. + 0.018):
        TX0[2, 3] = h_door/2. + 0.03
    
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = TX0[0, 3] + 0.01
    box_pose.pose.position.y = TX0[1, 3]
    box_pose.pose.position.z = TX0[2, 3]
    box_name = 'cabinet_box'
    scene.add_box(box_name, box_pose, size=(0.3, w_door, h_door))

    TA0 = TX0 @ np.linalg.inv(TXA)
    np.save(os.path.join(package_path, 'config', 'TA0.npy'), TA0)

    # TX0 = TTB @ TCT @ TAC @ Tz

    # init_pose = Pose()
    # init_pose.position.x = TAC[0, 2]
    # init_pose.position.y = TAC[1, 2]
    # init_pose.position.z = TAC[2, 2] + 0.5

    # q = quaternion_from_matrix(TAC)

    init_pose = Pose()
    init_pose.position.x = TX0[0, 3]
    init_pose.position.y = TX0[1, 3]
    init_pose.position.z = TX0[2, 3]

    q = quaternion_from_matrix(TX0)

    init_pose.orientation.x = q[0]
    init_pose.orientation.y = q[1]
    init_pose.orientation.z = q[2]
    init_pose.orientation.w = q[3]

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        _ = spawn_urdf_model_proxy(model_name, model_xml, '/', init_pose, 'world')
        rospy.loginfo('URDF model spawned successfully')

        # q = quaternion_from_matrix(TA0)
        # while not rospy.is_shutdown():
        #     stf_door_link = TransformStamped()
        #     stf_door_link.header.stamp = rospy.Time.now()
        #     stf_door_link.header.frame_id = 'world'
        #     stf_door_link.child_frame_id = 'door'
        #     stf_door_link.transform.translation.x = TA0[0, 3]
        #     stf_door_link.transform.translation.y = TA0[1, 3]
        #     stf_door_link.transform.translation.z = TA0[2, 3]
        #     stf_door_link.transform.rotation.x = q[0]
        #     stf_door_link.transform.rotation.y = q[1]
        #     stf_door_link.transform.rotation.z = q[2]
        #     stf_door_link.transform.rotation.w = q[3]
        #     broadcaster.sendTransform(stf_door_link)

    except rospy.ServiceException as e:
        rospy.logerr('Failed to spawn URDF model: %s' % e)


def move_model_joint(model_name, joint_name, joint_value):
    rospy.wait_for_service('/gazebo/set_model_configuration')

    try:
        set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        req = SetModelConfigurationRequest()
        req.model_name = model_name
        req.urdf_param_name = "robot_description"  # Assuming your URDF is loaded with the parameter name "robot_description"

        req.joint_names = [joint_name]
        req.joint_positions = [joint_value]

        _ = set_model_configuration(req)
        rospy.loginfo(f"Joint {joint_name} set to {joint_value} for model {model_name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to set joint configuration: {e}")


def detect_model(cfg_path, rgb_save_path, depth_save_path, ply_save_path):
    detector = RVLPYDDDetector.PYDDDetector()
    detector.create(cfg_path)

    for filename in os.listdir(ply_save_path):
        ply_path = os.path.join(ply_save_path, filename)
        detector.add_mesh(ply_path)

    for filename in os.listdir(rgb_save_path):
        rgb_path = os.path.join(rgb_save_path, filename)
        detector.add_rgb(rgb_path)

    ao = detector.detect()
    print(ao)
    

    # Prepare for saving
    ao_js = {}
    for key in ao:
        if isinstance(ao[key], np.ndarray):
            ao_js[key] = ao[key].tolist()
        else:
            ao_js[key] = ao[key]
    
    print(ao_js)

    with open(os.path.join(package_path, 'config', 'ao.txt'), 'w') as f:
        f.write(json.dumps(ao_js))
    
    return ao


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    node_name = 'detect_cabinet_model_node'
    rospy.init_node(node_name)

    scene = moveit_commander.PlanningSceneInterface()

    cfg_path = rospy.get_param('/rvl_cfg_path')
    rgb_save_path = rospy.get_param('/rgb_save_path')
    depth_save_path = rospy.get_param('/depth_save_path')
    ply_save_path = rospy.get_param('/ply_save_path')
    b_load_model = rospy.get_param('/load_model')

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('ao_manipulation')

    TCT = np.load(os.path.join(package_path, 'config', 'TCT_0.npy'))
    TTB = np.load(os.path.join(package_path, 'config', 'TTB_0.npy'))
    TB0 = np.load(os.path.join(package_path, 'config', 'TB0_0.npy'))

    listener = tf.TransformListener()
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    build_model_sub = rospy.Subscriber('/build_model', Bool, callback)
    pus_operation_pub = rospy.Publisher('/push_operation', Bool, queue_size=1)
    rospy.spin()

