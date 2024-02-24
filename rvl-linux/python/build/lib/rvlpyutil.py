import numpy as np

def rotx(q):
    c = np.cos(q)
    s = np.sin(q)
    return np.array([[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]])

def roty(q):
    c = np.cos(q)
    s = np.sin(q)
    return np.array([[c, 0.0, s, 0.0], [0.0, 1.0, 0.0, 0.0], [-s, 0.0, c, 0.0], [0.0, 0.0, 0.0, 1.0]])

def rotz(q):
    c = np.cos(q)
    s = np.sin(q)
    return np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

def transl(t):
    T = np.eye(4)
    T[:3,3] = t
    return T

def skew(x):
    Y = np.zeros((x.shape[0], 3, 3))
    Y[:,0,1] = -x[:,2]
    Y[:,0,2] = x[:,1]
    Y[:,1,0] = x[:,2]
    Y[:,1,2] = -x[:,0]
    Y[:,2,0] = -x[:,1]
    Y[:,2,1] = x[:,0]
    return Y

def angle_axis_to_rotmx(k, q):
    cq = np.cos(q)
    sq = np.sin(q)
    cqcomp = 1.0 - cq
    kxy = k[0] * k[1] * cqcomp
    kyz = k[1] * k[2] * cqcomp
    kzx = k[2] * k[0] * cqcomp
    return np.array([[k[0] * k[0] * cqcomp + cq, kxy - k[2] * sq, kzx + k[1] * sq, 0.0],
    [kxy + k[2] * sq, k[1] * k[1] * cqcomp + cq, kyz - k[0] * sq, 0.0],
    [kzx - k[1] * sq, kyz + k[0] * sq, k[2] * k[2] * cqcomp + cq, 0.0],
    [0.0, 0.0, 0.0, 1.0]])

def rotmx_to_angle_axis(R):
    k = 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)
    if k > 1.0:
        theta = 0.0
        axis = np.zeros(3)
    elif k < -1.0:
        theta = np.pi
        axis = np.zeros(3)
    theta = np.arccos(k)
    k = 0.5 / np.sin(theta)	
    axis = np.array([k * (R[2, 1] - R[1, 2]), k * (R[0, 2] - R[2, 0]), k * (R[1, 0] - R[0, 1])])
    axis = axis / np.linalg.norm(axis)
    return axis, theta

def inv_transf(T):
    invT = np.eye(4)
    invT[:3,:3] = T[:3,:3].T
    invT[:3,3] = -T[:3,:3].T @ T[:3,3]
    return invT

def homogeneous(points):
    return np.concatenate((points, np.ones((points.shape[0], 1))), axis=1)