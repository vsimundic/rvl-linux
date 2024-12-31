import numpy as np

def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    T = np.eye(4)
    T[:3,:3] = np.array([[c, -s, 0],
                         [s, c, 0],
                        [0, 0, 1]])
    return T

T0_B = np.load('T0_B.npy')

indices_list = [19, 22, 38, 43, 51, 59, 67, 74, 81, 83, 87, 89, 93, 118, 122, 149, 158, 159, 174, 187, 192, 214, 215, 257, 260, 264, 279, 313, 348, 392, 396, 398, 411, 426, 436, 471, 494, 515, 525, 546, 548, 550, 555, 558, 570, 579, 583, 592, 611, 655, 677, 683, 711, 724, 729, 763, 770, 775, 779, 787, 788, 797, 806, 813, 826, 829, 866, 875, 890, 905, 908, 928, 939, 940, 941, 970, 974, 979, 986, 988, 993, 999]


idx = indices_list[0]

T0_w = np.load('cabinet_%d_T0_w.npy' % idx)
Tw_e = np.load('cabinet_%d_Tw_e.npy' % idx)
T0_w2 = np.load('cabinet_%d_T0_w2.npy' % idx)
Tw_e2 = np.load('cabinet_%d_Tw_e2.npy' % idx)

Te_B = np.linalg.inv(T0_B) @ T0_w2 @ Tw_e2

Te_B_reshaped = Te_B.flatten()

# T0_w_rot = T0_w @ rot_z(-0.46873939461030362) @ Tw_e
# print(T0_w)
# print('')
# print(Tw_e)
# print('')
# print(T0_w_rot)
# print('')
# print(Te_B)


# print(Te_B)
for i in range(12): # without last row
    print(Te_B_reshaped[i], end=' ')
print('')