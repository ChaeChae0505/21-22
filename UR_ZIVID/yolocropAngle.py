
import cv2
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from PIL import Image
import math

global pts # for storing points
pts = []

class cal_cr:
    # center and angle and rotate robot angle
    def convert_robot(Rx,Ry,deg):
        rad = math.pi * (deg / 180)
        rx = math.cos(rad)*Rx - math.sin(rad)*Ry
        ry = math.sin(rad)*Rx - math.cos(rad)*Ry
        print("rx,ry",rx,ry)
        return rx, ry
    def convert_movej(deg):
        rad = math.pi * (deg / 180)
        return rad
    # Load the image
    def img_angle(IMG,thr1, thr2,result):
        check = []
        check2 = []

        img = cv2.imread(IMG)
        img_gray = Image.open(IMG).convert('L')
        img_array = np.array(img_gray)

        # Was the image there?
        if img is None:
            print("Error: File not found")
            exit(0)

        cv2.imshow('Input Image', img)

        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Convert image to binary
        _, bw1 = cv2.threshold(img_array, thr1, 255, cv2.THRESH_BINARY)# | cv.THRESH_OTSU)


        area_true_check = 0
        # Find all the contours in the thresholded image
        while 1:
            _, bw1 = cv2.threshold(img_array, thr1, 255, cv2.THRESH_BINARY)  # | cv.THRESH_OTSU)
            contours, _ = cv2.findContours(bw1, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) #cv.CHAIN_APPROX_NONE
            for i, c in enumerate(contours):
                # Calculate the area of each contour
                area = cv2.contourArea(c)
                print(area)
                if area > 800 and area < 1500:
                    area_true = area
                    area_true_check = 1
                    print(1)
                    break
            if area_true_check == 0 :
                if thr1 < 60:
                    print(1)

                elif thr1 > 60 :
                    thr1 = thr1 - 5
                    print(2)
                else :
                    print("Not find strain gauge")
                    break
            elif area_true_check == 1 :
                print("thr" , thr1)
                break
        print("c",  c)
        rect = cv2.minAreaRect(c)
        print("area",area_true)
        print(rect)
        box = cv2.boxPoints(rect)
        box = np.int0(box) # box 는 1:Left, 2:Top, 3:Bottom, 4:Right
        print("box",box, rect)

        # Retrieve the key parameters of the rotated bounding box
        center = (int(rect[0][0]), int(rect[0][1]))
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = rect[2]
        print("angle : ", angle, width, height)
        if width < height:
            angle = 90 - angle
            num = height
            print("1", angle)
        else:
            angle = - angle
            num = width
            print("2", angle)
        angle2 = int(angle)
        # if you want to write the image about object angle text box
        label = str(center)+ "&" + str(angle2) + " degrees"
        size_t = img.shape
        textbox = cv2.rectangle(img, (0, 0),
                               (img.shape[1], 20+int(0.1*img.shape[0])), (255, 255, 255), -1)
        cv2.putText(img, label, (10, 10+int(0.1*img.shape[0])),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


        cv2.circle(img, (center),1,(0,255,0),2)
        cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

        angle_check = center
        print("  Rotation Angle: " + str(angle) + " degrees")



        # center line 비교하기
        # bw

        _, bw2 = cv2.threshold(img_array, thr2, 255, cv2.THRESH_BINARY)
        while 1:
            for i in range(5,int(num/2)):
                x1, y1 = ((int(center[0] + i * np.cos(angle / 180 * np.pi)),
                                      int(center[1] - i * np.sin(angle / 180 * np.pi))))
                x2, y2 = (int(center[0] - i * np.cos(angle / 180 * np.pi)),
                             int(center[1] + i * np.sin(angle / 180 * np.pi)))
                check.append(bw2[y1][x1])
                check2.append(bw2[y2][x2])
            print(bw2.shape)

            # 평균 값 뽑기
            m_check, m_check2 = np.mean(check), np.mean(check2)
            print(m_check, m_check2)
            if m_check > m_check2:
                mid_point = (int(center[0] + 50 * np.cos(angle / 180 * np.pi)),
                         int(center[1] - 50 * np.sin(angle / 180 * np.pi)))
                cv2.line(img, (center), mid_point, (255, 0, 0), 2)
                print("위")
                break
            elif m_check < m_check2:
                mid_point = (int(center[0] - 50 * np.cos(angle / 180 * np.pi)),
                          int(center[1] + 50 * np.sin(angle / 180 * np.pi)))
                cv2.line(img, (center), mid_point, (0, 255, 0), 2)
                print("아래")
                break
            else:
                thr2 = thr2 - 10
                print("thresh", thr2)
                _, bw2 = cv2.threshold(img_array, thr2, 255, cv2.THRESH_BINARY)
                cv2.imshow("bw2", bw2)

                continue


        deg = angle





        cv2.imshow('Output Image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Save the output image to the current directory
        cv2.imwrite(result, img)
        return center[0], center[1], deg


if __name__ == "__main__":
    test_convey()
