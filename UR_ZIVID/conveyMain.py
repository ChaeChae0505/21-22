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
from conveyAngle import Temp_cap,cal_cr
from conveyCap import capture as cc



if __name__ == "__main__":
    # @ : robot motion , * : camera motion
    pts = []
    PATH = 'data/211122convey3/'
    min_x = 0
    min_y = 0

    ## @ picking pose
    pick_up = 0.50
    pick_z = 0.3855
    ## @ placing  list
    px = [0.3675, 0.3675 ,0.3675, 0.3675, 0.3675, 0.3675]  # robot placing list_x
    py = [-0.11547, -0.13, -0.145, -0.160, -0.175, -0.190]  # robot placing list_y
    pz_up = 0.50
    pz_down = 0.3525
    prx, pry, prz = 0.0, 3.14, 0.0

    # * open zivid camera
    app = zivid.Application()
    camera = app.connect_camera()

    # @* get strain gauge
    for i in range(6):
        IMG_DIR = PATH + 'Image'+str(i)+'.png'
        TEMP_DIR = PATH + 'Temp'+str(i)+'.png'
        RESULT_DIR = PATH + 'Result'+str(i)+'.png'
        FINAL_DIR = PATH + 'Final'+str(i)+'.png'
        #=====================================================#

        # @ Robot picking position
        robot.pick_position()

        # * capture하는 부분
        cc.png(camera, IMG_DIR)

        img = cv2.imread(IMG_DIR, 0)
        if i == 0 :
            roi = Temp_cap(min_x, min_y, IMG_DIR, TEMP_DIR)
            cv2.namedWindow('image',  flags=cv2.WINDOW_FREERATIO)
            cv2.setMouseCallback('image', roi.draw_roi)
            print("[INFO] Click the left button: select the point, right click: delete the last selected point, click the middle button: determine the ROI area")
            print("[INFO] Press ‘S’ to determine the selection area and save it")
            print("[INFO] Press ESC to quit")
            while True:
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                if key == ord("s"):
                    saved_data = {
                        "ROI": pts
                    }
                    print("pts", pts)
                    cv2.imwrite(TEMP_DIR, saved_data)
                    print("[INFO] ROI coordinates have been saved to local.")
                break
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        # * roi 지정해 줘서 해도 되는데!!
            max_x, max_y, min_x, min_y = roi.result()
        elif i > 0 :
            max_x, max_y, min_x, min_y = max_x, max_y, min_x, min_y
            temp = img.copy()
            temp = temp[min_y:max_y, min_x:max_x]
            cv2.imwrite(TEMP_DIR, temp)

        cx, cy, deg = cal_cr.img_angle(TEMP_DIR, 80, 200,RESULT_DIR)




        # point 주면 로봇 pose 나옴
        cX_Real, cY_Real = min_x+cx, min_y+cy
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

        # @ robot picking place
        # 보정하였음 None 값 처리 해주기 if 넣어서
        print("end",picking_point_robot)
        pose_ro = robot.posej()
        r_deg = pose_ro[5] * 180 / math.pi  # robot 기준 deg

        # @ 0.01mm 단위에서 반올림 , target points
        x,y,z = round(picking_point_robot[0]*0.001,6),round(picking_point_robot[1]*0.001,6), 0.420 #0.156532

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
        robot.getModjRZ(rad) # check!!!

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
