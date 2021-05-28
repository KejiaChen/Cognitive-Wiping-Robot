import cv2
import numpy as np
import os


def read_base_poses_from_file(filename):
    T_list = []
    R_list = []
    t_list = []
    t_2d_list = []
    with open(filename, 'r') as f:
        for line_id, line in enumerate(f):
            line = line.replace("]", "")
            line = line.replace("[", "")
            row = line.strip("\n").split(",")
            row = np.array([float(x) for x in row], dtype=np.float32)
            A_i = row.reshape((4, 4))
            A_i = A_i.T
            T_list.append(A_i)
            R_list.append(A_i[0:3, 0:3])
            t_list.append(A_i[0:3, 3])
            t_2d_list.append(A_i[0:2, 3])

    return t_2d_list

def read_cam_poses_from_file(filename):
    p_list = []
    with open(filename, 'r') as f:
        for line_id, line in enumerate(f):
            line = line.replace("]]", "")
            line = line.replace("[[", "")
            row = line.strip("\n").split(" ")
            # print(row)
            row = np.array([float(x) for x in row], dtype=np.float32)
            p_i = row.reshape((2, 1))
            p_list.append(p_i/1000)

    return p_list


cali_path = os.path.realpath(__file__)
cali_dir, filename = os.path.split(cali_path)
# points_in_cam = [np.array([252, 201])/1000,
#                  np.array([302, 196])/1000,
#                  np.array([350, 189])/1000,
#                  np.array([261, 246])/1000,
#                  np.array([313, 237])/1000,
#                  np.array([365, 229])/1000,
#                  np.array([269, 297])/1000,
#                  np.array([326, 287])/1000,
#                  np.array([384, 279])/1000]
points_in_cam = read_cam_poses_from_file(os.path.join(cali_dir, '9_point_board_cam.txt'))
points_in_base = read_base_poses_from_file(os.path.join(cali_dir, 'pose_list_9_points_board.txt'))

transform_cam_base, _ = cv2.estimateAffinePartial2D(np.array(points_in_cam), np.array(points_in_base))
R_cam_base = transform_cam_base[0:2, 0:2]
t_cam_base = transform_cam_base[0:2, 2]
print(transform_cam_base)

# test
point_cam = np.array([292, 150])/1000
point_cam = point_cam.reshape((2, 1))
point_base = np.array([0.5158, -0.04])
print(R_cam_base.dot(point_cam) + t_cam_base.reshape([2, 1]))
print(point_base)

