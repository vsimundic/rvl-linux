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

def perturbation_rot(pert_angle_deg):
    u = np.random.rand(3)
    u = u / np.linalg.norm(u)
    pert_angle_rad = np.deg2rad(pert_angle_deg)
    R_pert = angle_axis_to_rotmx(u, pert_angle_rad)
    return R_pert

def rot_size(R):
    return 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1.0)

def rot_angle(R):
    return np.arccos(rot_size(R))
	
def base(A):
	m = A.shape[0]
	n = A.shape[1]
	B = np.zeros(m).astype('int')
	T = np.concatenate((A, np.eye(m)), 1)
	all_raw_idx = np.arange(m)
	for i in range(m):
		B[i] = np.argmax(np.abs(T[i,:n]))
		T[i,:] /= T[i,B[i]]
		if m > 1:
			non_pivot_row = (all_raw_idx != i)
			T[non_pivot_row,:] = T[non_pivot_row,:] - T[non_pivot_row,B[i],np.newaxis] * T[np.newaxis,i,:]
	return B, T
	
def feasible_solution(A, b, x0, verbose=False):
	m = A.shape[0]
	n = A.shape[1]

	x = x0.copy()

	K = np.squeeze(A@x <= b, 1)
	J = np.zeros(m).astype('bool')
	feasible_solution_exists = True

	if np.count_nonzero(K) < m:
		while np.count_nonzero(K) < m:
			K_ = np.logical_not(K)
			k = np.argmax(K_)
			r = np.count_nonzero(J)
			if r == 0:
				v = -A[k,:,np.newaxis]
			else:
				B, T = base(A[J,:])				
				B_ = np.ones(n+r).astype('bool')
				B_[B] = False
				B_[n:] = False
				M = T[:,B_]
				N = T[:,n:]
				B_ = B_[:n]
				a_kB = A[np.newaxis,k,B]
				a_kB_ = A[np.newaxis,k,B_]
				c1 = a_kB @ M - a_kB_
				c2 = a_kB @ N
				zero_e_J = np.squeeze(c2 < 0, 0)
				if r == n and np.count_nonzero(zero_e_J) == n:
					feasible_solution_exists = False
					break
				else:
					v_B_ = c1.T
					e_J = c2.T
					e_J[zero_e_J,:] = 0
					v = np.zeros((n,1))
					v[B,:] = -M @ v_B_ - N @ e_J
					v[B_,:] = v_B_
					J_prev = J.copy()
					J[np.nonzero(J)] = zero_e_J
			w = A @ v
			s_k = (b[k,:,np.newaxis] - A[np.newaxis,k,:] @ x) / w[k,:,np.newaxis]
			intersect = (w[:,0] > 1e-10)
			K_intersect = np.logical_and(K, intersect)
			free_to_target = True
			if np.count_nonzero(K_intersect) > 0:
				s_K = np.zeros((m,1))
				s_K[K_intersect,:] = (b[K_intersect,:] - A[K_intersect,:] @ x) / w[K_intersect,:]
				K_intersect_positive = np.squeeze(s_K > 0, 1)
				if np.count_nonzero(K_intersect_positive) > 0:
					s_tmp = 2.0 * np.max(s_K) * np.ones(m)
					s_tmp[K_intersect_positive] = s_K[K_intersect_positive,0]
					j = np.argmin(s_tmp)
					s_j = s_K[j,:]
					if s_j[0] < s_k[0]:
						free_to_target = False
			if free_to_target:
				K[k] = True
				J[k] = True
				s = s_k
			else:
				J[j] = True
				s = s_j
			if s < 1e-10:
				J = np.logical_or(J, J_prev)
			x += s * v
			K = np.logical_or(K, np.squeeze(A@x <= b))
			if verbose:
				print('k=%d' % k)
				print('move dist: %f' % s)
				print('v:', np.squeeze(v))
				if r > 0:
					print('c1:', c1)
					print('c2:', c2)
					print('zero e_J:', zero_e_J)
					print('T:', T)
				if free_to_target:
					print('point on k')
				else:
					print('point on %d in K' % j)
					print('distance to k=%f' % np.squeeze(A[np.newaxis,k,:]@x-b[k]))			
				print('J: ', np.nonzero(J))
				print('K: ', np.nonzero(K))
				print(' ')
		if np.count_nonzero(K) == m:
			maxe = np.max(A@x-b)
			if maxe > 1e-10:
				print('Error! False feasible solution!')
				
	return x, feasible_solution_exists