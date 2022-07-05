#%%
from matplotlib import pyplot as plt
import cv2
from PIL import Image
import math
import numpy as np


##############################################################################################
#이미지읽기, 그레이화, 소벨
path = 'initial.png'
img_1 = Image.open(f'{path}').convert('L') ## @@@여기다가 yolo로 crop된 이미지 넣으면 됩니당(그레이 변환된걸로)@@@
Crop = np.array(img_1, 'uint8')

img_sobel_x = cv2.Sobel(Crop, cv2.CV_64F, 1, 0, ksize=5) ## width 측정용
img_sobel_x = cv2.convertScaleAbs(img_sobel_x)

img_sobel_y_k1 = cv2.Sobel(Crop, cv2.CV_64F, 0, 1, ksize=1) ## Height 측정용
img_sobel_y_k1 = cv2.convertScaleAbs(img_sobel_y_k1)

img_sobel_y = cv2.Sobel(Crop, cv2.CV_64F, 0, 1, ksize=5) ## Water 경계면 측정용
img_sobel_y = cv2.convertScaleAbs(img_sobel_y)


###############################################################################################
#변수 지정
Vertical_Center = []
Horizontal_Center = []
Vertical_Left = []
Vertical_Right = []

column = len(img_sobel_y)
row = len(img_sobel_y[0])
center = [int(column/2), int(row/2)]
########################################################################################
# 사진 중앙 수평, 수직 라인 그려서 sobel 값 추출
# segmentation center points 찾아서 
for index in range(row):
    Value = int(img_sobel_x[int(column/2)][index])
    if Value >= 200:
        Value = 255
    else:
        Value = 0
    Horizontal_Center.append(Value)

for index in range(column):
    Value = int(img_sobel_y[index][int(row/2)])
    if Value >= 200:
        Value = 255
    else:
        Value = 0
    Vertical_Center.append(Value)

########################################################################################

# Left,Right 계산 (병의 왼쪽, 오른쪽 경계)
for index in range(center[1],0,-1):
    point = Horizontal_Center[index]
    if point == 255:
        left_point=index
        break

for index in range(center[1],row,1):
    point = Horizontal_Center[index]
    if point == 255:
        right_point=index
        break
########################################################################################
## 검출한 병 왼쪽 오른쪽 지점 Vertical 라인 그려서 Sobel값 추출

for index in range(column):
    Value = int(img_sobel_y_k1[index][int(left_point+20)])
    if Value >= 30:
        Value = 255
    else:
        Value = 0
    Vertical_Left.append(Value)

for index in range(column):
    Value = int(img_sobel_y_k1[index][int(right_point-20)])
    if Value >= 30:
        Value = 255
    else:
        Value = 0
    Vertical_Right.append(Value)

########################################################################################
# Top,Bottom 계산

for index in range(center[0],0,-1):
    point = Vertical_Left[index]
    if point == 255:
        Top_point=index
        break

for index in range(center[0],column,1):
    point = Vertical_Left[index]
    if point == 255:
        Bottom_point=index
        break

Height = Bottom_point-Top_point
print("Height",Height)

for index in range(Bottom_point,0,-1):
    point = Vertical_Center[index]
    if point == 255:
        Water_point=index
        break

################################################################################
width = right_point - left_point
mm = 0.24
# widthmm = 39.20 / width
widthmm = mm * width
# heightmm = 102.20/ Height
heightmm = mm * Height
pixPmm = (widthmm + heightmm) / 2
radiusmm = widthmm / 2
Circle_Areamm = math.pi * (math.pow(radiusmm, 2))
Bottle_Volumemm = round((Circle_Areamm * heightmm) * 0.001, 2)

Radius = width / 2
Circle_Area = math.pi * (math.pow(Radius, 2))
Bottle_Volume = Circle_Area * Height
Water_percent = ((Bottom_point - Water_point) / Height) * 100
print("Width", width, widthmm, heightmm)
print("Bottle_Volume", Bottle_Volume, Bottle_Volumemm)
print("Water_Percentage", Water_percent, '%')
Bottle_Volume = round(Bottle_Volume)
Water_percent = round(Water_percent)
img1 = cv2.imread(IMG)
img2 = cv2.putText(img1, f'Bottle_volume', (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2, cv2.LINE_AA)

img2 = cv2.putText(img1, f'{Bottle_Volumemm}cm2', (10, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2, cv2.LINE_AA)
img2 = cv2.putText(img1, f'{Water_percent}%', (10, Water_point), cv2.FONT_HERSHEY_PLAIN, 5, (0, 0, 255), 5,
                   cv2.LINE_AA)

IMG = IMG.split('/')[-1]
cv2.imwrite(f'result_process.png', img2)
################################################################################



plt.imshow(img_sobel_x, cmap='gray')
plt.show()

plt.imshow(img_sobel_y_k1, cmap='gray')
plt.show()

plt.imshow(img_sobel_y, cmap='gray')
plt.show()

plt.plot(Horizontal_Center, color='r')
plt.show()

plt.subplot(3,1,1)
plt.plot(Vertical_Center, color='b')
plt.subplot(3,1,2)

plt.plot(Vertical_Left, color='g')
plt.subplot(3,1,3)

plt.plot(Vertical_Right, color='k')
plt.show()
