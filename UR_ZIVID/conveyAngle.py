
import cv2
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from PIL import Image
import math

global pts # for storing points
pts = []

class Temp_cap:
    def __init__(self, min_x, min_y, IMG_DIR, TEMP_DIR):
        self.min_x = min_x
        self.min_y = min_y
        self.IMG_DIR = IMG_DIR
        self.TEMP_DIR = TEMP_DIR
# :mouse callback function
    def draw_roi(self, event, x, y, flags, param):
        IMG_DIR = self.IMG_DIR
        TEMP = self.TEMP_DIR
        img = cv2.imread(IMG_DIR, 0)
        img2 = img.copy()
        if event == cv2.EVENT_LBUTTONDOWN:  # Left click, select point
            pts.append((x, y))
        if event == cv2.EVENT_RBUTTONDOWN:  # Right click to cancel the last selected point
            pts.pop()

        if event == cv2.EVENT_MBUTTONDOWN:  # center click
            mask = np.zeros(img.shape, np.uint8)
            points = np.array(pts, np.int32)
            points = points.reshape((-1, 1, 2))
            print(points)
            print(points.shape)
            x = points[0:,0,0]
            y = points[0:,0,1]
            print("x:",x, "y", y)
            self.max_x = max(x)
            self.max_y = max(y)
            self.min_x = min(x)
            self.min_y = min(y)
            # https://wikidocs.net/10063
            print(self.max_x, self.max_y, self.min_x, self.min_y)
            crop = img2[self.min_y:self.max_y, self.min_x:self.max_x]
            cy = (self.max_y - self.min_y)/2
            cx = (self.max_x - self.min_x)/2

            # Rotation = cv2.getRotationMatrix2D((cx,cy),angle,1)
            cv2.imwrite(TEMP, crop)
    #q
            mask = cv2.polylines(mask, [points], True, (255, 255, 255), 2)
            mask2 = cv2.fillPoly(mask.copy(), [points], (255, 255, 255))  # for ROI
            mask3 = cv2.fillPoly(mask.copy(), [points], (0, 255, 0))  # for displaying images on the desktop

            show_image = cv2.addWeighted(src1=img, alpha=0.8, src2=mask3, beta=0.2, gamma=0)

            cv2.imshow("mask", mask2)
            cv2.imshow("show_img", show_image)

            ROI = cv2.bitwise_and(mask2, img)
            cv2.imshow("ROI", ROI)
            cv2.waitKey(0)

        if len(pts) > 0:
            # Draw the last point in pts
            cv2.circle(img2, pts[-1], 3, (0, 0, 255), -1)

        if len(pts) > 1:
            #
            for i in range(len(pts) - 1):
                cv2.circle(img2, pts[i], 5, (0, 0, 255), -1)  # x ,y is the coordinates of the mouse click place
            cv2.line(img=img2, pt1=pts[i], pt2=pts[i + 1], color=(255, 0, 0), thickness=2)

        cv2.imshow('image', img2)

        return self.min_x, self.min_y
    def result(self):
        max_x = self.max_x
        max_y = self.max_y
        min_x = self.min_x
        min_y = self.min_y
        pts.clear()
        return max_x, max_y, min_x, min_y
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


def Repeat():
    min_x, min_y = 0,0

    # IMG = "Image.png"
    for i in range(5):
        DIR = './Data/'
        IMG_DIR = DIR + str(i)+'.png'
        TEMP_DIR = DIR + 'temp'+str(i)+'.jpg'
        RESULT = DIR + 'final'+str(i)+'.png'
        img = cv2.imread(IMG_DIR, 0)
        if i == 0:
            roi = Temp_cap(min_x, min_y, IMG_DIR, TEMP_DIR)
            cv2.namedWindow('image', flags=cv2.WINDOW_FREERATIO)
            cv2.setMouseCallback('image', roi.draw_roi)
            print(
                "[INFO] Click the left button: select the point, right click: delete the last selected point, click the middle button: determine the ROI area")
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
        elif i > 0:
            max_x, max_y, min_x, min_y = max_x, max_y, min_x, min_y
            temp = img.copy()
            temp = temp[min_y:max_y, min_x:max_x]
            cv2.imwrite(TEMP_DIR, temp)

        cx, cy, deg = cal_cr.img_angle(TEMP_DIR, 80, 200, i)


def test_convey():
    min_x, min_y = 0,0
    i = 93 #30
    PATH = 'data/211122convey3/'
    IMG_DIR = PATH + 'Image' + str(i) + '.png'
    TEMP_DIR = PATH + 'Temp' + str(i) + '.png'
    RESULT_DIR = PATH + 'Result' + str(i) + '.png'
    FINAL_DIR = PATH + 'Final' + str(i) + '.png'
    roi = Temp_cap(min_x, min_y, IMG_DIR, TEMP_DIR)
    cv2.namedWindow('image', flags=cv2.WINDOW_FREERATIO)
    cv2.setMouseCallback('image', roi.draw_roi)
    print(
        "[INFO] Click the left button: select the point, right click: delete the last selected point, click the middle button: determine the ROI area")
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

    cx, cy, deg = cal_cr.img_angle(TEMP_DIR, 80, 200,i, RESULT_DIR)


if __name__ == "__main__":
    test_convey()
