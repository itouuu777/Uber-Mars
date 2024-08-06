import serial
from datetime import datetime
from picamera2 import Picamera2
import cv2
import numpy as np
import time

ser = serial.Serial('/dev/ttyS0', 115200)
buffer = ""

def makecontent():
    now = datetime.now()
    formatted_date = now.strftime("%Y-%m-%d %H:%M:%S")
    return formatted_date

def cam_chosei():
    with Picamera2() as camera:
        try:
            camera.resolution = (640, 480)
            camera.start()
            
            while True:
                im = camera.capture_array()
                
                img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 0, 0])
                upper_red = np.array([30, 255, 255])
                
                frame_mask_red = cv2.inRange(hsv, lower_red, upper_red)

                red_contours, red_hierarchy = cv2.findContours(frame_mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                #小さいやつはじく（いらんか？）
                red_contours = [x for x in red_contours if cv2.contourArea(x) > 100]
                if len(red_contours) > 0:
                    #最大値返す
                    max_red_contour = max(red_contours, key=cv2.contourArea)

                    #モーメントの計算
                    M = cv2.moments(max_red_contour)
                    #重心の計算
                    if M["m00"] != 0:
                        center_mass = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    else:
                        center_mass = (0, 0)

                    # 画像の垂直中央線
                    vertical_center_x = im.shape[1] // 2
                    
                    # 重心と垂直中央線の描画
                    cv2.circle(img, center_mass, 5, (255,0 ,0), -1)
                    cv2.putText(img, f"Center of Mass: {center_mass}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    chuo = 0
                    if point_centor(center_mass[0], vertical_center_x):
                        print("Point is near vertical center line!")
                        ser.write(b'centor\n')
                        chuo = 1
                        break
                    elif center_mass[0] < vertical_center_x:
                        ser.write(b'left\n')
                        time.sleep(2)
                        chuo = 0
                    elif center_mass[0] > vertical_center_x:
                        ser.write(b'right\n')
                        time.sleep(2)
                        chuo = 0
                else:
                    ser.write(b'none\n')  # 物体が見つからない場合、右に移動し続ける
                
                # 画面に画像を表示（デバッグ用）
                cv2.imshow('Camera View', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            cv2.destroyAllWindows()

        return chuo

def point_centor(x, vertical_center_x, threshold=5):
    return abs(x - vertical_center_x) < threshold

def enough_purple(im):
    img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_purple = np.array([125, 50, 50])  
    upper_purple = np.array([150, 255, 255])  
    
    frame_mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)

    purple_contours, _ = cv2.findContours(frame_mask_purple, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return len(purple_contours) > 0

def par():
    with Picamera2() as camera:
        try:
            camera.resolution = (640, 480)
            camera.start()
            kaihi = 0

            while True:
                im = camera.capture_array()

                if enough_purple(im):
                    print("Purple object is present in the image.")
                    ser.write(b'para\n')
                    kaihi = 0
                    break
                else:
                    print("Purple object is not present in the image.")
                    ser.write(b'non\n')
                    kaihi = 1
                    break  
                    
        finally:
            cv2.destroyAllWindows()

    return kaihi

def enough_red(im, threshold=0.5):
    img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0, 0, 0])
    upper_red = np.array([30, 255, 255])
    
    frame_mask_red = cv2.inRange(hsv, lower_red, upper_red)
    
    red_contours, _ = cv2.findContours(frame_mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(red_contours) > 0:
        red_area = np.sum(frame_mask_red > 0)
        total_area = im.shape[0] * im.shape[1]
        red_area_ratio = red_area / total_area

        return red_area_ratio > threshold

    return False


def goal():
    with Picamera2() as camera:
        try:
            camera.resolution = (640, 480)
            camera.start()
            go = 0
            time.sleep(5)

            while True:
                im = camera.capture_array()

                if enough_red(im):
                    print("Red area is above 50% of the image.")
                    ser.write(b'arrival\n')
                    go = 0
                    break
                else:
                    print("Red area is below 50% of the image.")

                    ser.write(b'zensin\n')
                    time.sleep(2)
                    go = 1
                     
                    
        finally:
            cv2.destroyAllWindows()

    return go

with open("log.csv", "a", encoding='utf-8') as file:
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            buffer += data
            if '\n' in buffer:
                lines = buffer.split('\n')
                for line in lines[:-1]:
                    print("Complete line received: ", line)
                    file.write(makecontent() + ',' + line + '\n')
                    if line.strip() == "start_cam":
                        print("Starting cam_chosei()...")
                        cam_chosei()
                        print("Finished cam_chosei()")
                    elif line.strip() == "par_sta":
                        par()
                        print("Finished par")
                    elif line.strip() == "start_goal":
                        goal()
                        print("Finished_goal")
                buffer = lines[-1]  # バッファに残った最後の部分を保持
