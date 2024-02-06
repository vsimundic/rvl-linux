import numpy as np
import math
#import pytransform3d
import geometry_msgs.msg
import pytransform3d.transformations

def pose_to_matrix(pose):
    #q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    #rotation = transforms3d.quaternions.quat2mat(q)
    #result = np.array([
    #    [rotation[0][0], rotation[0][1], rotation[0][2], pose.position.x],
    #    [rotation[1][0], rotation[1][1], rotation[1][2], pose.position.y],
    #    [rotation[2][0], rotation[2][1], rotation[2][2], pose.position.z],
    #    [0             , 0             , 0             , 1]
    #])


    #Using pytransform3d package we can use following function for transformations from position and quaternion form that we get from robot and transformation matrix form that we need for calculations
    #https://dfki-ric.github.io/pytransform3d/_apidoc/pytransform3d.transformations.transform_from_pq.html#pytransform3d.transformations.transform_from_pq
    pq = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    result = pytransform3d.transformations.transform_from_pq(pq)
    return np.array(result)

def matrix_to_pose(matrix):
    pq = pytransform3d.transformations.pq_from_transform(matrix)
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pq[0]
    pose.position.y = pq[1]
    pose.position.z = pq[2]
    pose.orientation.w = pq[3]
    pose.orientation.x = pq[4]
    pose.orientation.y = pq[5]
    pose.orientation.z = pq[6]
    return pose

if __name__ == "__main__":
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 1
    pose.position.y = 7
    pose.position.z = 13
    pose.orientation.w = 1
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    print(pose)
    matrix = pose_to_matrix(pose)
    print(matrix)
    pose = matrix_to_pose(matrix)
    print(pose)
    matrix = pose_to_matrix(pose)
    print(matrix)
    pose = matrix_to_pose(matrix)
    print(pose)
