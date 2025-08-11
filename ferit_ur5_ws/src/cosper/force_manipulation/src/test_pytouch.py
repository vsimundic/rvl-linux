import RVLPYDDManipulator as rvlpy
import os
import numpy as np
from gazebo_push_open.cabinet_model import Cabinet
import open3d as o3d

base_dir = '/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection'

rvl_ddmanipulator_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Cupec_real_robot.cfg'
rvl_touch_cfg = '/home/RVLuser/rvl-linux/RVLMotionDemo_Touch_Cupec.cfg'
rvl_manipulator = rvlpy.PYDDManipulator()
rvl_manipulator.create(rvl_ddmanipulator_cfg)
py_touch = rvl_manipulator.py_touch
py_touch.create(rvl_touch_cfg)
touch_is_init = True
a_tool = 0.019 + 0.005
b_tool = 0.064 / 2.0
c_tool = 0.007 + 0.005
d_tool = 0.049 / 2.0
h_tool = 0.02706
T_tool_6 = np.load(os.path.join(base_dir, 'T_tool_6.npy'))
py_touch.create_simple_tool(a_tool, b_tool, c_tool, d_tool, h_tool, T_tool_6)

camera_fu = 597.9033203125
camera_fv = 598.47998046875
camera_uc = 323.8436584472656
camera_vc = 236.32774353027344
camera_w = 640
camera_h = 480
py_touch.set_camera_params(camera_fu, camera_fv, camera_uc, camera_vc, camera_w, camera_h)
touch_a = 0.4
touch_b = 0.0
touch_c = 0.005

session_set = False

data_cabinet = "0,0,0.018,0.3940131366252899,0.5324208736419678,0.0,-0.19700656831264496,-8.024078879649423,-0.07260735332965851,-0.9973350763320923,0.0071348994970321655,-0.46467289328575134,0.02749716490507126,-0.8850553631782532,0.8825005888938904,-0.06757692247629166,-0.4654310941696167,-0.28441351652145386,-0.03671291470527649,0.9964064359664917,-0.7125028175589725,-0.7009394938493924,0.031993138837611,0.7015597836487493,-0.712456123709312,0.014837174792416495,0.012393745892386815,0.033016628405370486,0.9993779551858738,0.07985568832444188,0.08353177092684139,0.059728059869907146,-0.7187796226280281,0.6952252538047039,-0.004207204166070622,0.31537993681577436,0.3314461350892099,0.8892013017244765,0.6195896622352475,0.637812908309804,-0.4574968245185991,0.11793998899774578,-0.5136733713954765,0.5678154531915404"
# data_cabinet = "0,0,0.018,0.3940131366252899,0.5324208736419678,0.0,-0.19700656831264496,-8.024078879649423,0.06732076925113664,-0.9977058666096502,0.0071348994970321655,-0.46396181882193743,-0.03763538903489094,-0.8850553631782532,0.8832934729317368,0.05627228914502177,-0.4654310941696167,-0.28441351652145386,-0.03671291470527649,0.9964064359664917,-0.7125028175589725,-0.7009394938493924,0.031993138837611,0.7015597836487493,-0.712456123709312,0.014837174792416495,0.012393745892386815,0.033016628405370486,0.9993779551858738,0.07985568832444188,0.08353177092684139,0.059728059869907146,-0.7187796226280281,0.6952252538047039,-0.004207204166070622,0.31537993681577436,0.3314461350892099,0.8892013017244765,0.6195896622352475,0.637812908309804,-0.4574968245185991,0.11793998899774578,-0.5136733713954765,0.5678154531915404"
data_split = data_cabinet.split(',')
s = np.array([float(data_split[2]), float(data_split[3]), float(data_split[4])])
r = np.array([float(data_split[5]), float(data_split[6])])
state_angle = float(data_split[7])
R_A_C = np.array(data_split[8:17], dtype=float).reshape(3, 3)
t_A_C = np.array(data_split[17:20], dtype=float)
R_C_E = np.array(data_split[20:29], dtype=float).reshape(3, 3)
t_C_E = np.array(data_split[29:32], dtype=float)
R_E_0 = np.array(data_split[32:41], dtype=float).reshape(3, 3)
t_E_0 = np.array(data_split[41:44], dtype=float)

T_A_C = np.eye(4)
T_A_C[:3, :3] = R_A_C
T_A_C[:3, 3] = t_A_C
T_C_E = np.eye(4)
T_C_E[:3, :3] = R_C_E
T_C_E[:3, 3] = t_C_E
T_E_0 = np.eye(4)
T_E_0[:3, :3] = R_E_0
T_E_0[:3, 3] = t_E_0


data_touch = "0,0,0,0.8683170647976994,-0.02788034531801734,0.49522536417883084,-0.0911294380989523,0.9724564076750416,0.21453195725467716,-0.48756630368892057,-0.23141136859865236,0.8418597733532716,0.16265386736371062,-0.19123837832999357,0.8223805889156381,0.305325709459429,0.3910064533715857,0.8682684864515527,0.04499463317576226,-8.024078879649423"
data_touch_split = data_touch.split(',')

type_touch = int(data_touch_split[2])
R_Ek_E = np.array(data_touch_split[3:12], dtype=float).reshape(3, 3)
t_Ek_E = np.array(data_touch_split[12:15], dtype=float)
V = np.array(data_touch_split[15:18], dtype=float)
t = float(data_touch_split[18])
state_angle_touch = float(data_touch_split[19])
T_Ek_E = np.eye(4)
T_Ek_E[:3, :3] = R_Ek_E
T_Ek_E[:3, 3] = t_Ek_E
b_miss = True if type_touch == 2 else False


T_A_0 = T_E_0 @ T_C_E @ T_A_C
T_A_E = T_C_E @ T_A_C
cabinet_model = Cabinet(door_params=np.array([s[1], s[2], s[0], 0.4]),
                        r=r,
                        axis_pos=-1,
                        T_A_S=T_A_0,
                        save_path=None,
                        has_handle=False)
cabinet_model.change_door_angle(state_angle)
cabinet_mesh = cabinet_model.create_mesh()
cabinet_mesh.transform(np.linalg.inv(cabinet_model.T_A_O_init))
cabinet_mesh.transform(T_A_E)
axis_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
axis_rf.transform(T_A_E)

gripper_mesh_path = os.path.join('/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply')
gripper_mesh = o3d.io.read_triangle_mesh(gripper_mesh_path)
T_G_E = np.load(os.path.join(base_dir, 'T_G_6.npy'))
gripper_mesh.transform(T_G_E)
T_Ek_0 = T_E_0 @ T_Ek_E
gripper_mesh.transform(T_Ek_E)

origin_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])

T_tool_0 = T_Ek_0 @ T_tool_6
T_tool_E = T_Ek_E @ T_tool_6
tool_rf = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
tool_rf.transform(T_tool_E)

o3d.visualization.draw_geometries([cabinet_mesh, gripper_mesh, origin_rf, tool_rf, axis_rf])

if not session_set:
    py_touch.set_session_params(s[0], s[1], s[2], r[0], r[1], 
                                touch_a, touch_b, touch_c, state_angle,
                                T_C_E, T_A_C, T_E_0)
    session_set = True

py_touch.set_new_scene(s[0], s[1], s[2], r[0], r[1], 
                            touch_a, touch_b, touch_c, state_angle,
                            T_C_E, T_A_C, T_E_0)


py_touch.set_touch(T_Ek_E, V, t, b_miss)

py_touch.correct()