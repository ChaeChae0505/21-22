# Robot motion urx
import urx
from time import sleep
import os
import cv2
import math
from math import *
import threading
from queue import Queue

rob = urx.Robot("172.28.60.2")
rob.set_tcp((0, 0, 0, 0, 0, 0))
rob.set_payload(1, (1, 1, 1))

class conveyRobot():
    def pick_position():
        # picking position
        v_robot = 0.4
        # rob.movel([0.3913779577559793, 0.10920118604975466, 0.604758627856639, 2.179068571958713, -2.2523161307599815,
        #            0.012589190823600287], v_robot, v_robot)
        rob.movel([0.49337659734614897, 0.026409202670308475, 0.38, 2.179063900436179, -2.252277059057348, 0.012411254321776572], v_robot, v_robot)

    def capture_position():
        # last capture position
        v_robot = 0.4
        rob.movel([0.3503492142279018, -0.25609611886789857, 0.4285349691429946, 2.1789560119734523, -2.249886760849419, -0.021765801583985746], v_robot, v_robot)

    # x, y, z ,rx, ry, rz 다 바꿀 수 있는 거
    def modXYZR3(x, y, z, rx, ry, rz):
        v_robot = 0.4
        rob.movel([x, y, z, rx, ry, rz], v_robot, v_robot)  # initial setting
        # [-0.26206, -0.42048, 0.22998, 0, 3.14, 0]

    def modXYZ(x, y, z):  # robot x, y, z
        v_robot = 0.3
        rob.movel([x, y, z, 2.179068571958713, -2.2523161307599815, 0.012589190823600287], v_robot, v_robot)  # initial setting
        # [-0.26206, -0.42048, 0.22998, 0, 3.14, 0]

    def getModZ(z):  # change z
        v_robot = 0.3
        get = rob.getl(rob)
        rob.movel([get[0], get[1], z, get[3], get[4], get[5]], v_robot, v_robot)

    def getModXYZ(x, y, z):
        v_robot = 0.3
        get = rob.getl(rob)
        rob.movel([x, y, z, get[3], get[4], get[5]], v_robot, v_robot)

    def getModR3(rx, ry, rz):  # robot x, y, z
        v_robot = 0.4
        get = rob.getl(rob)
        rob.movel([get[0], get[1], get[2], rx, ry, rz], v_robot, v_robot)  # initial setting
        # [-0.26206, -0.42048, 0.22998, 0, 3.14, 0]


    def getModjRZ(rz):
        get = rob.getj(rob)
        v_robot = 0.2
        rob.movej((get[0], get[1], get[2], get[3], get[4], rz), v_robot, v_robot)
        print(rz)


    # @ check the current pose
    def pose():
        get = rob.getl(rob)
        return get

    def posej():
        get = rob.getj(rob)
        return get

    # @ Tool IO
    def pick():  # picking
        rob.set_tool_voltage(24)
        rob.set_digital_out(9, True)

    def place():  # placing
        rob.set_tool_voltage(0)
        rob.set_digital_out(8, True)

    def pickfinish():  # only finish suction
        rob.set_digital_out(9, False)
        rob.set_digital_out(8, False)

    # @ Robot
    def finish():  # finish robot
        rob.set_digital_out(9, False)
        rob.set_digital_out(8, False)
        rob.close()
if __name__ == "__main__":
    ## @ placing  list
    px = [0.3675, 0.3675 ,0.3675, 0.3675, 0.3675, 0.3675]  # robot placing list_x
    py = [-0.11547, -0.13, -0.145, -0.160, -0.175, -0.190]  # robot placing list_y
    pz = 0.35406
    prx, pry, prz = 0.0, 3.14, 0.0
    # 0.4208813924540272,-0.18832506981924146
    conveyRobot.pick_position()
    # conveyRobot.capture_position()
    get = conveyRobot.pose()
    print(get)