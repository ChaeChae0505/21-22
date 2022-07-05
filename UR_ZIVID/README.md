# ZIVID - UR calibration
## $ Requirements
- zivid sdk install
- zivid-python-sample -> pip install -r requirements.txt
- UR-Camera-Calibration
- Yolov5
- rtde
-------------------------------------------------------------

# CODE 설명
### Multi
- 베드위에 여러개를 둔 후 작동을 시키는 것 

### single
- 한베드에 한개 하지만 연속된 동작 

### ROI
- 첫 1회만 ROI 지정하면 한베드에 한개 하지만 연속된 동작 

------------
## 문제점
- 빛에 강건하지 못 하다 -> HSV 채널을 쓰거나, 딥러닝을 쓰는 방법


## Naming 규칙
- Conveyor belt 위에 동작하는 것들은 conveyMain.py 식으로 작성
- test 파일들은 제일 첫 대문자 ex) ConveyorTest.py
- 내용이 변하지 않는 파일은 첫 글자 대문자 
