import numpy as np
import math

def calculate(tool_E, markers_B, images):
    #C:
    # num_markers = 6
    # first_marker_idx = 4
    # tool_E[2] = 0.285
    # markers_B_ = np.zeros((num_markers,4))
    # for marker_idx in range(first_marker_idx,first_marker_idx+num_markers):
    #     markers_B_[marker_idx-first_marker_idx,:] = markers_B[marker_idx]["position"] @ tool_E
    # markers_c_B = np.mean(markers_B_, axis=0)
    # markers_n_B = markers_B_ - np.expand_dims(markers_c_B, 0)
    # markers_n_B = markers_n_B[:,:3]
    # markers_M_B = np.expand_dims(markers_n_B, 2) @ np.expand_dims(markers_n_B, 1)
    # markers_M_B = np.sum(markers_M_B, 0) / num_markers
    # markers_eigvals, markers_eigvect = np.linalg.eig(markers_M_B)
    # min_marker_eigval_idx = np.argmin(markers_eigvals)
    # markers_normal = markers_eigvect[:,min_marker_eigval_idx]

    # markers_C = np.zeros((num_markers,4))
    # for index, image in enumerate(images):
    #     for marker in image["markers"]:
    #         marker_idx = marker["id"]
    #         markers_C[marker_idx-first_marker_idx,:] = marker["position"]
 
    #positional_vector is list of lists of calculated positional vectors for each marker in image
    #calculate by using (1) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    positional_vectors = list()
    for index, image in enumerate(images):
        image_vector = list()
        for marker in image["markers"]:
            pos_v = np.linalg.inv(image["position"]) #image["position"] #np.linalg.inv(image["position"]) #TODO:inverse(?)
            print(pos_v)
            vector = np.dot(pos_v, markers_B[marker["id"]]["position"])
            print(vector)
            vector = np.dot(vector, tool_E)
            print(vector)
            image_vector.append({"image": index, "marker":marker["id"], "position": vector})
        positional_vectors.append(image_vector)

    print("positional_vectors={}".format(positional_vectors))
    #get average value for vectors in space E (calculated in step 1)
    #calculate by using (2) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    count = 0
    sum = np.zeros_like(positional_vectors[0][0]["position"])
    for image in positional_vectors:
        for marker in image:            
            sum = sum + marker["position"]
            count = count + 1
    avg_E = sum/count
    #print("avg_E={}".format(avg_E))
    
    #get average value for vectors in space C
    #calculate by using (3) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    count = 0
    sum = np.zeros_like(images[0]["markers"][0]["position"])
    for index, image in enumerate(images):
        for marker in image["markers"]:
            sum = sum + marker["position"]
            count = count + 1
    avg_C = sum/count
    #print("avg_C={}".format(avg_C))

    #calculate a_ij and b_ij by using (4) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    a = list()
    for image in positional_vectors:
        for marker in image:
            vector = marker["position"] - avg_E
            a.append({"image": marker["image"], "marker": marker["marker"], "vector": vector})
    #print("a={}".format(a))

    b = list()
    for index, image in enumerate(images):
        for marker in image["markers"]:
            vector = marker["position"] - avg_C
            b.append({"image": index, "marker": marker["id"], "vector":vector})
    #print("b={}".format(b))

    # for index in range(0, len(a)):
    #     len_a = np.linalg.norm(a[index]["vector"][:3])
    #     len_b = np.linalg.norm(b[index]["vector"][:3])
    #     debug = len_a / len_b

    # calculate scale factor s by using (5) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    sum_b = 0
    for vector in b:
        sum_b = sum_b + np.linalg.norm(vector["vector"], ord=2) ** 2

    sum_a = 0
    for vector in a:
        sum_a = sum_a + np.linalg.norm(vector["vector"], ord=2) ** 2
    
    #s = math.sqrt(sum_b/sum_a)
    s = math.sqrt(sum_a/sum_b)
    if(sum_b == sum_a):
        s = 1
    print("s = {}".format(s))

    #create S matrix (do not confuse it with s scale factor) by using (8) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    #print("len(a)={}".format(len(a)))
    #print("len(b)={}".format(len(b)))

    #if len(a) != len(b):
    #    printf("Length of a and b are not equal")
    #    exit(1)

    S = np.zeros((3,3))
    for index in range(0, len(a)):
        #print("a[{}]={}".format(index, a[index]))
        #print("b[{}]={}".format(index, b[index]))
        #uzmemo samo [x y z]
        #value = np.matmul(a[index]["vector"][0:3], b[index]["vector"][0:3].T)
        #S = S + value
        #C:
        M = np.expand_dims(a[index]["vector"][0:3],1) @ np.expand_dims(b[index]["vector"][0:3],0)
        S = S + M
        
    print("S matrix")
    print(S)
    #create N matrix from S matrix by using (7) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    x = 0
    y = 1
    z = 2
    N = np.array([
        [(S[x][x]+S[y][y]+S[z][z]), S[y][z]-S[z][y],          S[z][x]-S[x][z],          S[x][y]-S[y][x]],
        [ S[y][z]-S[z][y],         (S[x][x]-S[y][y]-S[z][z]), S[x][y]+S[y][x],          S[z][x]+S[x][z]],
        [ S[z][x]-S[x][z],          S[x][y]+S[y][x],        (-S[x][x]+S[y][y]-S[z][z]), S[y][z]+S[z][y]],
        [ S[x][y]-S[y][x],          S[z][x]+S[x][z],          S[y][z]+S[z][y],        (-S[x][x]-S[y][y]+S[z][z])]
    ])

    print("N matrix")
    print(N)

    #get eigenvalues and eigenvectors from matrx N
    (N_eigvals, N_eigvect) = np.linalg.eigh(N)
    max_index = np.argmax(N_eigvals)
    print(max_index)
    print(N_eigvals)
    print(N_eigvect)
    
    #rotation quaternion is vector from highest eigenvalue
    q = N_eigvect[:,max_index]
    print("q")
    print(q)
    #create E_R_C rotation matrix by using (9) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    o = 0
    x = 1
    y = 2
    z = 3
    E_R_C = np.array([
        [1.0-2.0*(q[y]**2+q[z]**2),   2.0*(q[x]*q[y]-q[z]*q[o]), 2.0*(q[x]*q[z]+q[y]*q[o])],
        [2.0*(q[x]*q[y]+q[z]*q[o]),   1.0-2.0*(q[x]**2+q[z]**2), 2.0*(q[y]*q[z]-q[x]*q[o])],
        [2.0*(q[x]*q[z]-q[y]*q[o]),       2.0*(q[y]*q[z]+q[x]*q[o]), 1.0-2.0*(q[x]**2+q[y]**2)]
    ]).T

    print("E_R_C")
    print(E_R_C)

    #create E_t_C translation vector by using (10) from EYE-IN-HAND CAMERA CALIBRATION Technical Report v1.0
    E_t_C = avg_E[0:3] - s * np.dot(E_R_C, avg_C[0:3])
    print("E_t_C")
    print(E_t_C)

    return (s, E_R_C, E_t_C)

def calculate2(tool_E_init, markers_rob, images, num_iterations):
    max_marker_id = -1
    num_points = 0
    for index, image in enumerate(images):
        num_points += len(image["markers"])
        for marker_cam in image["markers"]:
            max_marker_id = np.maximum(max_marker_id, marker_cam["id"])

    pt_cam = np.zeros((num_points, 4))
    pt_idx = 0
    for index, image in enumerate(images):
        for marker_cam in image["markers"]:
            pt_cam[pt_idx,:] = marker_cam["position"]
            pt_idx += 1
    
    tool_E = tool_E_init
    for iteration in range(num_iterations):
        markers_rob_B = np.zeros((max_marker_id+1, 4))
        for marker_id in range(1,max_marker_id+1):
            if marker_id in markers_rob:
                markers_rob_B[marker_id,:] = markers_rob[marker_id]["position"] @ tool_E       
        pt_rob = np.zeros((num_points, 4))
        pt_idx = 0
        for index, image in enumerate(images):
            T_BEimg = np.linalg.inv(image["position"])
            for marker_cam in image["markers"]:
                pt_rob[pt_idx,:] = T_BEimg @ markers_rob_B[marker_cam["id"],:]
                pt_idx += 1
        T_CE = pc_registration(pt_rob, pt_cam)
        pt_cam_E = T_CE @ np.expand_dims(pt_cam, 2)
        E = np.sum((pt_rob - pt_cam_E[:,:,0])**2)
        pt_idx = 0
        tool_E_new = np.zeros((4,1))    
        E = 0.0    
        for index, image in enumerate(images):
            T_BEimg = np.linalg.inv(image["position"])
            for marker_cam in image["markers"]:
                pt_cam_Erob = np.linalg.inv(markers_rob[marker_cam["id"]]["position"]) @ image["position"] @ pt_cam_E[pt_idx,:]
                tool_E_new += pt_cam_Erob
                E += np.sum((tool_E - pt_cam_Erob[:,0]) ** 2)
                pt_idx += 1
        tool_E_new = tool_E_new[:,0] / pt_idx
        tool_E_new[3] = 1.0
        tool_E = tool_E_new
    return T_CE, tool_E
    
def pc_registration(pca, pcb):
    num_pts = pca.shape[0]
    if pcb.shape[0] != num_pts:
        print('Error: Point clouds must have the same number of points!')
        return np.zeros((4,4))
    a_mean = np.mean(pca[:,:3], axis=0)
    b_mean = np.mean(pcb[:,:3], axis=0)
    a = pca[:,:3] - a_mean
    b = pcb[:,:3] - b_mean
    s = np.sqrt(np.sum(a**2) / np.sum(b**2))
    M = np.expand_dims(a,2) @ np.expand_dims(b,1)
    S = np.sum(M, axis=0)
    x = 0
    y = 1
    z = 2
    N = np.array([
        [(S[x][x]+S[y][y]+S[z][z]), S[y][z]-S[z][y],          S[z][x]-S[x][z],          S[x][y]-S[y][x]],
        [ S[y][z]-S[z][y],         (S[x][x]-S[y][y]-S[z][z]), S[x][y]+S[y][x],          S[z][x]+S[x][z]],
        [ S[z][x]-S[x][z],          S[x][y]+S[y][x],        (-S[x][x]+S[y][y]-S[z][z]), S[y][z]+S[z][y]],
        [ S[x][y]-S[y][x],          S[z][x]+S[x][z],          S[y][z]+S[z][y],        (-S[x][x]-S[y][y]+S[z][z])]
    ])
    (N_eigvals, N_eigvect) = np.linalg.eigh(N)
    max_index = np.argmax(N_eigvals)
    q = N_eigvect[:,max_index]
    o = 0
    x = 1
    y = 2
    z = 3
    R = np.array([
        [1.0-2.0*(q[y]**2+q[z]**2),   2.0*(q[x]*q[y]-q[z]*q[o]), 2.0*(q[x]*q[z]+q[y]*q[o])],
        [2.0*(q[x]*q[y]+q[z]*q[o]),   1.0-2.0*(q[x]**2+q[z]**2), 2.0*(q[y]*q[z]-q[x]*q[o])],
        [2.0*(q[x]*q[z]-q[y]*q[o]),       2.0*(q[y]*q[z]+q[x]*q[o]), 1.0-2.0*(q[x]**2+q[y]**2)]
    ]).T
    t = a_mean - s * R @ b_mean
    T = np.identity(4)
    T[:3,:3] = s * R
    T[:3,3] = t

    return T
    
    
