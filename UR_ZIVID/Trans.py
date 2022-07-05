import urx
import math

import numpy as np
import cv2
from PIL import Image
import datetime
import zivid
from pathlib import Path
from sample_utils.paths import get_sample_data_path
from sample_utils.settings_from_file import get_settings_from_yaml
from scipy.spatial.transform import Rotation
import warnings , os
import rtde.rtde as rtde


class Tran():
    def _get_frame_and_transform_matrix(con:rtde, camera: zivid.Camera, settings: zivid.Settings):
        """Capture image with Zivid camera and read robot pose

        Args:
            con: Connection between computer and robot
            camera: Zivid camera
            settings: Zivid settings

        Returns:
            Zivid frame
            4x4 tranformation matrix
        """
        rob = urx.Robot("172.28.60.2")

        frame = camera.capture(settings)
        robot_pose = np.array(rob.getl())

        translation = robot_pose[:3] * 1000
        rotation_vector = robot_pose[3:]
        rotation = Rotation.from_rotvec(rotation_vector)
        transform = np.eye(4)
        transform[:3, :3] = rotation.as_matrix()
        transform[:3, 3] = translation.T

        return frame, transform
    def _save_zdf_and_pose(save_dir: Path, transform: np.array):
        """Save data to folder

        Args:
            save_dir: Directory to save data
            image_num: Image number
            frame: Point cloud stored as .zdf
            transform: 4x4 transformation matrix
        """

        file_storage = cv2.FileStorage(str(save_dir / f"E:/0.DASIMA/211223_msi/po.yaml"), cv2.FILE_STORAGE_WRITE)
        file_storage.write("PoseState", transform)
        file_storage.release()

        ######################################################################################################################\
    def Transformation_Matrix(point_in_camera_frame) :
        def _assert_valid_matrix(file_name):
            """Check if YAML file is valid.

            Args:
                file_name: Path to YAML file.

            Returns None

            Raises:
                FileNotFoundError: If the YAML file specified by file_name cannot be opened.
                NameError: If the transformation matrix named 'PoseState' is not found in the file.
                ValueError: If the dimensions of the transformation matrix are not 4 x 4.
            """
            file_storage = cv2.FileStorage(str(file_name), cv2.FILE_STORAGE_READ)
            if not file_storage.open(str(file_name), cv2.FILE_STORAGE_READ):
                file_storage.release()
                raise FileNotFoundError(f"Could not open {file_name}")

            pose_state_node = file_storage.getNode("PoseState")

            if pose_state_node.empty():
                file_storage.release()
                raise NameError(f"PoseState not found in file {file_name}")

            shape = pose_state_node.mat().shape
            if shape[0] != 4 or shape[1] != 4:
                file_storage.release()
                raise ValueError(f"Expected 4x4 matrix in {file_name}, but got {shape[0]} x {shape[1]}")
        def _read_transform(transform_file):
            """Read transformation matrix from a YAML file.

            Args:
                transform_file: Path to the YAML file.

            Returns:
                transform: Transformation matrix.

            """
            file_storage = cv2.FileStorage(str(transform_file), cv2.FILE_STORAGE_READ)
            transform = file_storage.getNode("PoseState").mat()
            file_storage.release()

            return transform
        try:
            np.set_printoptions(precision=2)

            point_in_camera_frame = [point_in_camera_frame[0],point_in_camera_frame[1],point_in_camera_frame[2],1]
            eye_in_hand_transform_file =  "E:/0.DASIMA/211223_msi/transformation.yaml"
            robot_transform_file = "E:/0.DASIMA/211223_msi/po.yaml" #'pos.yaml'

            # Checking if YAML files are valid
            _assert_valid_matrix(eye_in_hand_transform_file)
            _assert_valid_matrix(robot_transform_file)

            # print("Reading camera pose in end-effector frame (result of eye-in-hand calibration)")
            transform_end_effector_to_camera = _read_transform(eye_in_hand_transform_file)

            # print("Reading end-effector pose in robot base frame")
            transform_base_to_end_effector = _read_transform(robot_transform_file)

            # print("Computing camera pose in robot base frame")
            transform_base_to_camera = np.matmul(transform_base_to_end_effector, transform_end_effector_to_camera)
            point_in_base_frame = np.matmul(transform_base_to_camera, point_in_camera_frame)

            print(f"Point coordinates in robot base frame: {point_in_base_frame[0:3]}")

            return point_in_base_frame[0:3]
        finally:
            pass
#
# point = np.array([0,0,0])
# Transformation_Matrix(point)
#
# if __name__ == "__main__":
#     # point 주면 로봇 pose 나옴
#     cX_Real, cY_Real = 1320, 650
#
#     try:
#         app = zivid.Application()
#         camera = app.connect_camera()
#         settings_file = Path() / get_sample_data_path() / f"D:/0.DASIMA/210915/Settings01.yml"
#         print(f"Configuring settings from file: {settings_file}")
#         settings = get_settings_from_yaml(settings_file)
#         host = '172.28.60.2'
#         con = rtde.RTDE(host, 30004)
#         con.connect()
#         connection_state = con.connect()
#         print(connection_state)
#         ##Process time data
#
#         while True:
#             # Get Transformation of current pos
#             frame, transform = _get_frame_and_transform_matrix(con, camera, settings)
#             save_dir = Path.cwd()
#             _save_zdf_and_pose(save_dir, transform)
#
#             depth_image = frame.point_cloud().copy_data("xyz")  # Get point coordinates as [Height,Width,3] float array
#             rgba = frame.point_cloud().copy_data("rgba")  # Get point colors as [Height,Width,4] uint8 array
#             # acquire box image
#             ## cX_Real, cY_Real : Point of the target (unit: pixel)
#             ## depth image : the numpy array has the real x,y,z value
#             picking_point_camera = [depth_image[cY_Real][cX_Real][0], depth_image[cY_Real][cX_Real][1],
#                                     depth_image[cY_Real][cX_Real][2]] # picking point coordinate from camera
#
#             ## transfrom picking point from camera to from robot
#             picking_point_robot = Transformation_Matrix(picking_point_camera)
#             print(picking_point_camera, picking_point_robot) # 어떤 변환으로 된건지 확인하기 확인해서 뎁스 같은거 고정해서 주거나 z 축을 고정해서 줘야할 듯 xy rotation 각도만 받거나!!!!
#             if picking_point_robot is not None:
#                 break
#
#
#
#
#     finally:
#         pass
#
#     print("end",picking_point_robot)