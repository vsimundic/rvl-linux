import numpy as np

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

def test1():
	A = np.random.rand(2,6)

	pivot_idx = np.zeros(2).astype('int')
	A2 = np.zeros((2,8))
	A2[:,:6] = A
	A2[:,6:] = -np.eye(2)
	pivot_idx[0] = np.argmax(np.abs(A2[0,:6]))
	A2[0,:] /= A2[0,pivot_idx[0]]
	A2[1,:] -= A2[1,pivot_idx[0]] * A2[0,:]
	pivot_idx[1] = np.argmax(np.abs(A2[1,:6]))
	A2[1,:] /= A2[1,pivot_idx[1]]
	A2[0,:] -= A2[0,pivot_idx[1]] * A2[1,:]

	other_idx = np.setdiff1d(np.arange(6),pivot_idx)
	A3 = A2[:,other_idx]
	A4 = A2[:,6:]

	for i in range(30):
		x2=2 * np.random.rand(4,1) - 1
		s=np.random.rand(2,1)
		x1=-A3@x2-A4@s
		x=np.zeros((6,1))
		x[pivot_idx,:] = x1
		x[other_idx,:] = x2
		print(A@x)
		
def test2():
	n = 6
	m = 10
	load_from_file = False
	verbose = False

	for iTest in range(1000):
		print('Test %d' % iTest)
		print(' ')
		if load_from_file:
			data = np.load('data.npy')
			m = data.shape[0]
			n = data.shape[1]-1
			A = data[:,:n]
			b = data[:,n,np.newaxis]
		else:
			A = 2.0 * np.random.rand(m, n) - 1.0
			b = 2.0 * np.random.rand(m, 1) - 1.0
			np.save('data', np.concatenate((A,b), 1))
		
		x = np.zeros((n,1))
		K = np.squeeze(A@x <= b, 1)
		J = np.zeros(m).astype('bool')

		if np.count_nonzero(K) == m:
			print('Zero vector is feasible solution.')
			print(' ')
		else:
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
						print('No feasible solution exists.')
						print(' ')
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
					if free_to_target:
						print('point on k')
					else:
						print('point on %d in K' % j)
						print('distance to k=%f' % np.squeeze(A[np.newaxis,k,:]@x-b[k]))			
					print('J: ', np.nonzero(J))
					print('K: ', np.nonzero(K))
					debug = 0
			if np.count_nonzero(K) == m:
				print('Feasible solution is:')
				print(x)
				maxe = np.max(A@x-b)
				print('max(e)=%f' % maxe)
				print(' ')
				if maxe > 1e-10:
					print('Error! False feasible solution!')
					break
					
test2()				
					
					
					
					
					
		
