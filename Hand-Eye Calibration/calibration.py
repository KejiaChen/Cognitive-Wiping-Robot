import os
import cv2
import PIL
import numpy as np
import argparse
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial.transform import Rotation
from camera_handler import CameraHandler

# camera D435
CAMERA_INTRINSICS_MAT = np.array(
    [[615.9142456054688, 0.0, 327.5135498046875], [0.0, 616.4674682617188, 242.85317993164062], [0, 0, 1]], dtype=np.float32)
# camera 1
# CAMERA_INTRINSICS_MAT = np.array(
#     [[610.331, 0, 312.435], [0, 608.768, 246.99], [0, 0, 1]], dtype=np.float32)
# camera 2
# CAMERA_INTRINSICS_MAT = np.array(
# [[614.182, 0, 315.91], [0, 614.545, 244.167], [0, 0, 1]], dtype=np.float32)
CAMERA_DISTORTION_COEFF_MAT = np.array([0, 0, 0, 0, 0], dtype=np.float32)
ARUCO_NAME = cv2.aruco.DICT_4X4_50
MARKER_SIDE_LENGTH_MM = 0.153
MARKER_SEPRATION = MARKER_SIDE_LENGTH_MM/4


class HandEyeCalibration():
    """
    To solve the pose from camera to robot base X, we need to solve a function AX = XB,
    A = inv(T_base_ee_next) * T_base_ee_current,
    B = T_marker_cam_next * inv(T_marker_cam_current)
    """

    def __init__(self, camera_intrinsics_mat, camera_distortion_coeff_mat, mode):
        self.cam_intr_mat = camera_intrinsics_mat
        self.cam_dist_coeff_mat = camera_distortion_coeff_mat
        self.marker_side_length_mm = MARKER_SIDE_LENGTH_MM
        self.marker_separation = MARKER_SEPRATION
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_NAME)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.marker_to_cam_poses_list = []
        self.R_marker_to_cam_poses_list = []
        self.t_marker_to_cam_poses_list = []
        self.ee_to_base_poses_list = []
        self.R_ee_to_base_poses_list = []
        self.t_ee_to_base_poses_list = []
        self.realsense_handler = CameraHandler((640, 480), CAMERA_INTRINSICS_MAT, CAMERA_DISTORTION_COEFF_MAT )
        self.calibration_mode = mode

    def draw_marker(self):
        img = cv2.aruco.drawMarker(self.aruco_dict, 0, 700)
        plt.imshow(img, cmap=mpl.cm.gray)
        plt.show()

    def read_ee_to_base_poses_from_file(self, filename):
        self.ee_to_base_poses_list = []
        with open(filename, 'r') as f:
            for line_id, line in enumerate(f):
                line = line.replace("]", "")
                line = line.replace("[", "")
                row = line.strip("\n").split(",")
                row = np.array([float(x) for x in row], dtype=np.float32)
                A_i = row.reshape((4, 4))
                A_i = A_i.T
                self.ee_to_base_poses_list.append(A_i)
                self.R_ee_to_base_poses_list.append(A_i[0:3, 0:3])
                self.t_ee_to_base_poses_list.append(A_i[0:3, 3])

    def save_images_with_key(self, save_dir):
        self.realsense_handler.save_left_camera_images(save_dir)

    def get_images_list_from_dir(self, images_dir):
        assert(os.path.exists(images_dir))
        image_paths_list = sorted(os.listdir(images_dir))
        images_list = []
        for image_idx, image_name in enumerate(image_paths_list):
            image_path = os.path.join(images_dir, image_name)
            image = cv2.imread(image_path)
            images_list.append(image)
        return images_list

    def compute_A(self):
        A = np.zeros((4, (len(self.ee_to_base_poses_list)-1)*4))
        for pose_idx, (current_pose, next_pose) in enumerate(zip(self.ee_to_base_poses_list)):
            A_i = np.linalg.inv(next_pose) @ current_pose
            A[:, pose_idx * 4: pose_idx * 4 + 4] = A_i
        return A

    def compute_B(self):
        B = np.zeros((4, (len(self.marker_to_cam_poses_list)-1)*4))
        for pose_idx, (current_pose, next_pose) in enumerate(zip(self.marker_to_cam_poses_list[:-1], self.marker_to_cam_poses_list[1:])):
            B_i = next_pose @ np.linalg.inv(current_pose)
            B[:, pose_idx * 4: pose_idx * 4 + 4] = B_i
        return B

    def get_poses_from_images(self, images_list):
        self.marker_to_cam_poses_list = []
        for image_idx, image in enumerate(images_list):
            return_pose = self.get_pose_from_one_image(image)
            # print(image_idx)
            # print(return_pose)
            if return_pose is not None:
                self.marker_to_cam_poses_list.append(
                    return_pose)
                self.R_marker_to_cam_poses_list.append(
                    return_pose[0:3, 0:3]
                )
                self.t_marker_to_cam_poses_list.append(
                    return_pose[0:3, -1]
                )
        # print("done")

    def get_pose_from_one_image(self, image):
        marker_corners, marker_ids = self.detect_aruco_corners(image)
        frame_markers = cv2.aruco.drawDetectedMarkers(image.copy(), marker_corners, marker_ids)
        # plt.figure()
        # plt.imshow(frame_markers, origin="upper")
        # plt.show()

        # If use a single marker
        if self.calibration_mode == 's':
            rot_vec, trans_vec, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, self.marker_side_length_mm, self.cam_intr_mat, self.cam_dist_coeff_mat)
            # if rot_vec is not None && trans_vec is not None:
            # (rot_vec - trans_vec).any()  # get rid of that nasty numpy value array error
            cv2.aruco.drawDetectedMarkers(image, marker_corners)  # Draw A square around the markers
            # cv2.aruco.drawAxis(image, self.cam_intr_mat, self.cam_dist_coeff_mat, rot_vec, trans_vec, 0.01)  # Draw axis

            # Show images
            # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('RealSense', image)
            # k = cv2.waitKey(0)

        # If use an Aruco Board
        elif self.calibration_mode == 'b':
            rot_vec = None
            trans_vec = None
            board = cv2.aruco.GridBoard_create(3, 3, self.marker_side_length_mm, self.marker_separation,
                                           self.aruco_dict)
            success, rot_vec, trans_vec = cv2.aruco.estimatePoseBoard(
                marker_corners, marker_ids, board, self.cam_intr_mat, self.cam_dist_coeff_mat, rot_vec, trans_vec)

            if success > 0:
                axis_image = cv2.aruco.drawAxis(image.copy(), self.cam_intr_mat, self.cam_dist_coeff_mat, rot_vec, trans_vec, 0.05)
                # plt.figure()
                # plt.imshow(axis_image, origin="upper")
                # plt.show()
        else:
            print("Wrong mode: please choose between s and b")

        if rot_vec is None or trans_vec is None:
            return None

        rot_vec = rot_vec.reshape(3,)
        sci_rotation = Rotation.from_rotvec(rot_vec)
        rot_mat = sci_rotation.as_matrix()
        trans_mat = np.concatenate(
            [rot_mat, trans_vec.reshape((3, 1))], axis=1)
        trans_mat = np.concatenate(
            [trans_mat, np.array([0, 0, 0, 1]).reshape((1, 4))], axis=0)
        # print(trans_mat)
        # print("done")
        return trans_mat

    def detect_aruco_corners(self, image):
        marker_corners, marker_ids, rejected_candidates = cv2.aruco.detectMarkers(
            image, self.aruco_dict, parameters=self.detector_parameters)
        return marker_corners, marker_ids

    def test_opencv_key_press(self):
        cap = cv2.VideoCapture(0)
        counter = 0
        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret:
                cv2.imshow('frame', frame)
            k = cv2.waitKey(33)
            if k == ord('s'):
                cv2.imwrite('{0:06}.png'.format(counter), frame)
                counter += 1
            elif k == ord('q'):
                break

        cap.release()

if __name__ == '__main__':
    # calib = HandEyeCalibration(
        # CAMERA_INTRINSICS_MAT, CAMERA_DISTORTION_COEFF_MAT)
    # calib.save_images_with_key("./")
    # handler = CameraHandler((640, 480))
    # # handler.save_left_camera_images("./")
    # handler.save_right_camera_images("./")

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--pose', default='pose_list_0526_s1.txt',
                        type=str,
                        dest='pose_path',
                        help='path of pose_list')
    parser.add_argument('--image', default='images0526_s1',
                        type=str,
                        dest='image_path',
                        help='path of calibration images')
    parser.add_argument('--mode', default='s',
                        type=str,
                        dest='cali_mode',
                        help='use single marker (s) or marker board (b)')
    args = parser.parse_args()
    
    calib = HandEyeCalibration(
        CAMERA_INTRINSICS_MAT, CAMERA_DISTORTION_COEFF_MAT, args.cali_mode)

    # Get R,t ee_to_base
    # pose = '/home/kejia/wiping/Cognitive-Wiping-Robot/Hand-Eye Calibration/' + args.pose_path
    cali_path = os.path.realpath(__file__)
    # print(cali_path)
    cali_dir, filename = os.path.split(cali_path)
    pose_list = os.path.join(cali_dir, args.pose_path)
    calib.read_ee_to_base_poses_from_file(pose_list)
    T_ee_base = calib.ee_to_base_poses_list
    R_ee_base = calib.R_ee_to_base_poses_list
    t_ee_base = calib.t_ee_to_base_poses_list
    R_base_ee = []
    t_base_ee = []
    for r, t in zip(R_ee_base, t_ee_base):
        R_base_ee.append(r.T)
        t_base_ee.append(-r.T.dot(t))
    
    # Get R,T c_to_marker
    image_dir = os.path.join(cali_dir, args.image_path)
    img_list = calib.get_images_list_from_dir(image_dir)
    calib.get_poses_from_images(img_list)
    R_marker_cam = calib.R_marker_to_cam_poses_list
    t_marker_cam = calib.t_marker_to_cam_poses_list
    R_cam_marker = []
    t_cam_marker = []
    for r, t in zip(R_marker_cam, t_marker_cam):
        R_cam_marker.append(r.T) 
        t_cam_marker.append(-r.T.dot(t))

    # R_marker_ee, t_marker_ee = cv2.calibrateHandEye(R_ee_base, t_ee_base, R_cam_marker, t_cam_marker,
    #                                                       cv2.CALIB_HAND_EYE_ANDREFF)
    #
    # # T_marker_ee
    # T_marker_ee = np.concatenate(
    #     [R_marker_ee, t_marker_ee.reshape((3, 1))], axis=1)
    # T_marker_ee = np.concatenate(
    #     [T_marker_ee, np.array([0, 0, 0, 1]).reshape((1, 4))], axis=0)
    #
    # def calculate_final(index):
    #     # T_cam_marker
    #     T_cam_marker_i = np.concatenate(
    #         [R_cam_marker[index], t_cam_marker[index].reshape((3, 1))], axis=1)
    #     T_cam_marker_i = np.concatenate(
    #         [T_cam_marker_i, np.array([0, 0, 0, 1]).reshape((1, 4))], axis=0)
    #
    #     T_cam_base_i = T_ee_base[index].dot(T_marker_ee).dot(T_cam_marker_i)
    #     return T_cam_base_i
    #
    # T_cam_base = calculate_final(0)
    # print(calculate_final(0))
    # print(calculate_final(1))
    # print(calculate_final(2))

    # Another Method
    R_cam_to_base, t_cam_to_base = cv2.calibrateHandEye(R_base_ee[0:10], t_base_ee[0:10], R_marker_cam[0:10], t_marker_cam[0:10],
                                                          cv2.CALIB_HAND_EYE_ANDREFF)

    T_cam_base = np.concatenate(
                [R_cam_to_base, t_cam_to_base.reshape((3, 1))], axis=1)
    T_cam_base = np.concatenate(
        [T_cam_base, np.array([0, 0, 0, 1]).reshape((1, 4))], axis=0)

    print(T_cam_base)
    T_string = np.array2string(T_cam_base)
    if args.cali_mode == 'b':
        file = os.path.join(cali_dir, 'T_cam_base_board.txt')
    elif args.cali_mode == 's':
        file = os.path.join(cali_dir, 'T_cam_base.txt')
    with open(file, 'a') as f:
        f. write('%s\n' % T_string)
