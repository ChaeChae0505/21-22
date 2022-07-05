import cv2
from Trans import Tran
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
import time
from zivid import Application, Settings

from conveyCap import capture as cc
from conveyMotion import conveyRobot as robot


if __name__ == "__main__":
    robot.pick_position()
    app = Application()

    camera = app.connect_camera()
    cc.png(camera, "test.png")
    file_name  = "result.png"
    cX_Real, cY_Real = 1127, 481

    # point 주면 로봇 pose 나옴
    try:

        settings_file = Path() / get_sample_data_path() / f"D:/0.DASIMA/0.ZIVID_NEW/Settings01.yml"
        print(f"Configuring settings from file: {settings_file}")
        settings = get_settings_from_yaml(settings_file)
        host = '172.28.60.2'
        con = rtde.RTDE(host, 30004)
        con.connect()
        connection_state = con.connect()
        print(connection_state)
        ##Process time data

        while True:
            # Get Transformation of current pos
            # Trans.py 에 파일 있음
            frame, transform = Tran._get_frame_and_transform_matrix(con, camera, settings)
            save_dir = Path.cwd()
            Tran._save_zdf_and_pose(save_dir, transform)

            depth_image = frame.point_cloud().copy_data("xyz")  # Get point coordinates as [Height,Width,3] float array
            #211108 LCY  point cloud to 2D image
            rgba = frame.point_cloud().copy_data("rgba")  # Get point colors as [Height,Width,4] uint8 array
            bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
            cv2.imwrite(file_name, bgr)

            print("rgba",rgba)
            frame.save("result.zdf")
            # acquire box image
            ## cX_Real, cY_Real : Point of the target (unit: pixel)
            ## depth image : the numpy array has the real x,y,z value
            picking_point_camera = [depth_image[cY_Real][cX_Real][0], depth_image[cY_Real][cX_Real][1],
                                    depth_image[cY_Real][cX_Real][2]] # picking point coordinate from camera

            ## transfrom picking point from camera to from robot
            picking_point_robot = Tran.Transformation_Matrix(picking_point_camera)
            print("cam-robot",picking_point_camera, picking_point_robot) # 어떤 변환으로 된건지 확인하기 확인해서 뎁스 같은거 고정해서 주거나 z 축을 고정해서 줘야할 듯 xy rotation 각도만 받거나!!!!
            if picking_point_robot is not None:
                break





    finally:
        pass
    x,y = round(picking_point_robot[0]*0.001,6),round(picking_point_robot[1]*0.001,6)
    print(x, y)
    print(x+0.0005 , y + 0.0055)
    # @ robot picking place
    # 보정하였음 None 값 처리 해주기 if 넣어서
