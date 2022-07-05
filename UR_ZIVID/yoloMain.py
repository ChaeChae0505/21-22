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

from conveyMotion import conveyRobot as robot
from yolocropAngle import cal_cr
from conveyCap import capture as cc
import subprocess
import os


def startYolo(yolo):
    sp = subprocess.call(yolo, stdout=subprocess.PIPE, shell=True)

if __name__ == "__main__":


    path = 'yolov5/data/images/' # Yolo에서 꺼내서 할 수 있도록
    yolo = "python D:/0.DASIMA/210915_change/yolov5/detect.py --save-txt --save-crop --weights D:/0.DASIMA/210915_change/yolov5/best.pt --source"
    cropPath = './yolov5/runs/detect/exp'

    # * open zivid camera
    app = zivid.Application()
    camera = app.connect_camera()

    for i in range(6):
        IMG_DIR = f'{path}/{i}.jpg'

        ## Initial pose and capture
        robot.pick_position()
        cc.png(camera, IMG_DIR)

        ## Yolo
        yolo = f'{yolo} {IMG_DIR}'
        startYolo(yolo)

        ## Labels read
        j = 1
        cropPath = 'G:/0.DASIMA/210915_change/yolov5/runs/detect/exp'
        cropLabels = f'{cropPath}{i}/labels/'
        originalImage = f'{cropPath}{i}/'
        cropImage = f'{cropPath}{i}/crops/strain/{j}.jpg'
        result = 'G:/0.DASIMA/210915_change/result.jpg'

        img = Image.open(f'{originalImage}{j}.jpg').convert('L')
        img = np.array(img)
        size = img.shape

        # x initial, y initial
        label = open(f'{cropLabels}{j}.jpg.txt', mode='rt', encoding='utf-8')

        labels = label.readline()
        objectPoint = labels.split()

        centerX = float(objectPoint[1]) * size[1]
        centerY = float(objectPoint[2]) * size[0]
        width = float(objectPoint[3]) * size[1]
        height = float(objectPoint[4]) * size[0]
        initX = int(centerX - width / 2)
        initY = int(centerY - height / 2)

        label.close()

        ## crop image contour
        cx, cy, deg = cal_cr.img_angle(cropImage, 80, 200, result)
        cX_Real = initX + cx
        cY_Real = initY + cy

        print("Result:", cX_Real,cY_Real, min_x+cx, min_y+cy)
        try:

            settings_file = Path() / get_sample_data_path() / f"D:/0.DASIMA/210915/Settings01.yml"
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
                rgba = frame.point_cloud().copy_data("rgba")  # Get point colors as [Height,Width,4] uint8 array
                # acquire box image
                ## cX_Real, cY_Real : Point of the target (unit: pixel)
                ## depth image : the numpy array has the real x,y,z value
                picking_point_camera = [depth_image[cY_Real][cX_Real][0], depth_image[cY_Real][cX_Real][1],
                                        depth_image[cY_Real][cX_Real][2]] # picking point coordinate from camera

                ## transfrom picking point from camera to from robot
                picking_point_robot = Tran.Transformation_Matrix(picking_point_camera)
                print(picking_point_camera, picking_point_robot) # 어떤 변환으로 된건지 확인하기 확인해서 뎁스 같은거 고정해서 주거나 z 축을 고정해서 줘야할 듯 xy rotation 각도만 받거나!!!!
                if picking_point_robot is not None:
                    break

        finally:
            pass


        print("end", picking_point_robot)
        pose_ro = robot.posej()
        r_deg = pose_ro[5] * 180 / math.pi  # robot 기준 deg

        # @ 0.01mm 단위에서 반올림 , target points
        x, y, z = round(picking_point_robot[0] * 0.001, 6), round(picking_point_robot[1] * 0.001,
                                                                  6), 0.420  # 0.156532

        y = y + 0.0055
        print(x, y, z)
        time.sleep(2)

        if x < 0.6 or x > 0.40:
            # @ target points
            robot.modXYZ(x, y, z)
            print("good")
        else:
            robot.finish()
            print("stop")
            break

        if deg == -90 or deg == 90:
            deg = r_deg

        elif deg < 0 and deg > - 45:
            deg = -20
            print("change")

        elif deg < -45 and deg > -90:
            deg = r_deg - (90 + deg)
            print("check", deg)

        elif deg > 0 and deg < 90:
            deg = r_deg + (90 - deg)
            print("check", deg)

        rad = cal_cr.convert_movej(deg)

        # @ target angle
        robot.getModjRZ(rad)  # check!!!

        # @ target axis-z
        robot.getModZ(pick_z)

        # @ robot picking
        robot.pick()
        time.sleep(1)

        # @ up
        robot.getModZ(pick_up)
        time.sleep(2)

        # @ robot place z-axis 33cm
        robot.modXYZR3(px[i], py[i], pz_up, prx, pry, prz)

        time.sleep(2)
        robot.getModZ(pz_down)
        time.sleep(1)
        robot.place()

        time.sleep(3)
        # robot placing
        robot.getModZ(pz_up)
        robot.pickfinish()

    # @ robot capture
    robot.capture_position()
    time.sleep(3)

    # * Camera capture
    cc.final(camera, FINAL_DIR)

    robot.finish()
    time.sleep(2)




