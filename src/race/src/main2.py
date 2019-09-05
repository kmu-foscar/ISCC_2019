#!/usr/bin/env python

import cv2
import threading
# import Queue as que
import time
import numpy as np

# import roslib
import sys
import rospy
import math
import importlib
# import cPickle
# import genpy.message
# from rospy import ROSException
# import sensor_msgs.msg
# import actionlib
# import rostopic
# import rosservice
# from rosservice import ROSServiceException

from slidewindow import SlideWindow
from warper import Warper
from pidcal import PidCal

from race.msg import lane_info, drive_values
from std_msgs.msg import UInt8

import os

warper = Warper()
slidewindow  = SlideWindow()
pidcal = PidCal()

cv_image = None
ack_publisher = None
car_run_speed = 2

y_th = 125
w_th = 140
g_th = 20

lane_info_pub = rospy.Publisher('lane_info', lane_info, queue_size=1)
drive_values_pub = rospy.Publisher('control_value', drive_values, queue_size=1)
stop_line_pub = rospy.Publisher('stopline', UInt8, queue_size=1)

def auto_drive(steer):
    global car_run_speed

    w = 0

    if -30.0 < steer and steer < 30.0:
        w = 1
    else:
        w = 0.5

    if car_run_speed < 7.0*w:
        car_run_speed += 0.05*10
    else:
        car_run_speed -= 0.07*10

    # drive_value = drive_values()
    # drive_value.steering = steer*0.3
    # drive_value.throttle = 5*w
    # drive_values_pub.publish(drive_value)

    lane_info_ = lane_info()
    lane_info_.steering = steer
    lane_info_pub.publish(lane_info_)
    print('steer: ', steer)
    # print('speed: ', car_run_speed)

def main():
    # rospy.init_node('lane_detector_goni_node')
    # cap = cv2.VideoCapture("FINAL.avi")
    # cap = cv2.VideoCapture("FINAL2.avi")
    # cap = cv2.VideoCapture("FINAL5.avi")
    # cap = cv2.VideoCapture("TEST11.avi")
    # cap = cv2.VideoCapture("TEST30.avi")
    # cap = cv2.VideoCapture("TEST31.avi")
    # cap = cv2.VideoCapture("SunFeel.avi")

    cap = cv2.VideoCapture(0)
    cap.set(3,800)
    cap.set(4,448)

    while True:
        ret, img = cap.read()
        img1, x_location, steer = process_image(img)
        cv2.imshow('result', img1)

        # print("x_location : ",x_location)

        if x_location != None:
            # pid = round(pidcal.pid_control(int(x_location)), 6)

            # print("pid : ", pid)
            # print("degree : ", pid/0.074)
            # auto_drive(pid)
            auto_drive(steer)
            # print pid
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.waitKey(1) & 0xFF == ord('w'):
            cv2.waitKey(1000000)


      #out.write(img1)
      #out2.write(cv_image)

def stop_line(frame):

    out_img = np.dstack((frame, frame, frame)) * 255
    height = frame.shape[0]
    width = frame.shape[1]

	# inputImage = gray[y:y+h, x:x+w]

    x = 180
    w = 440
    y = 320
    h = 80

    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0

    cnt = 0

    roi_img = frame[y:y+h, x:x+w]

    lines = cv2.HoughLines(roi_img,1,np.pi/180,100)
    print(lines)

    stop_line_flag = UInt8()

    if lines is None:
        stop_line_flag.data = 0
        stop_line_pub.publish(stop_line_flag)
        return [-1, -1, -1, -1]
    else :
        for i in range(len(lines)):
            for rho, theta in lines[i]:
                tempTheta = theta/np.pi *180
                if(tempTheta > 88 and tempTheta < 92):
                    cnt = cnt + 1

                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 += int(x0 + 1000*(-b))
                    y1 += int(y0 + 1000*(a))
                    x2 += int(x0 - 1000*(-b))
                    y2 += int(y0 -1000*(a))
        if cnt != 0:
            stop_line_flag.data = 1
            stop_line_pub.publish(stop_line_flag)
            return [int(x1/cnt), int(y1/cnt + y), int(x2/cnt), int(y2/cnt + y)]
        else :
            stop_line_flag.data = 0
            stop_line_pub.publish(stop_line_flag)
            return [-1, -1, -1, -1]


def light_calc(frame):
    arr = np.array(frame)

    idx = [0,255]

    new_arr = np.delete(arr,idx)

    max_val = np.max(new_arr)
    min_val = np.min(new_arr)

    err = max_val - min_val
    bottom = int(255//err)

    # print(err)
    if err is not 0:
        frame = (frame - min_val)*bottom

    # print('min_val : ' ,min_val)
    # print('max_val : ', max_val )

    # cv2.imshow("dst",dst)

    return frame

def process_image(frame):

    global y_th
    global w_th

    # if cv2.waitKey(1) & 0xFF == ord('a'):
    #     y_th += 5
    #     print("y_th : ", y_th)
    # if cv2.waitKey(1) & 0xFF == ord('s'):
    #     y_th -= 5
    #     print("y_th : ",y_th)

    # blur
    kernel_size = 3
    blur = cv2.GaussianBlur(frame,(kernel_size, kernel_size), 0)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 0)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 0)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 0)
    blur = cv2.GaussianBlur(blur,(kernel_size, kernel_size), 0)

    # lab = cv2.cvtColor(blur,cv2.COLOR_BGR2LAB)
    # l1,a,b = cv2.split(lab)
    #
    # new_l1 = light_calc(l1)
    # new_a = light_calc(a)
    # new_b = light_calc(b)

    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    h1,s1,v = cv2.split(hsv)

    new_h1 = light_calc(h1)
    new_s1 = light_calc(s1)
    # new_v = light_calc(v)

    hls = cv2.cvtColor(blur,cv2.COLOR_BGR2HLS)
    h2,l2,s2 = cv2.split(hls)

    # new_h2 = light_calc(h2)
    new_l2 = light_calc(l2)
    # new_s2 = light_calc(s2)

    ret,binary_img = cv2.threshold(new_s1, y_th, 255, cv2.THRESH_BINARY)

    # cv2.imshow("bi",binary_img)

    ret1,binary_img1 = cv2.threshold(new_l2, w_th, 255, cv2.THRESH_BINARY)

    # cv2.imshow("bi1",binary_img1)

    ret2,binary_img2 = cv2.threshold(new_h1, g_th, 255, cv2.THRESH_BINARY)

    # cv2.imshow("bi2",binary_img2)

    img_mask = cv2.bitwise_or(binary_img,binary_img1)

    img_mask2 = cv2.bitwise_and(img_mask,~binary_img2)

    # img_result = cv2.bitwise_and(binary_img,binary_img,mask = img_mask)

    cv2.imshow("img_result",img_mask2)


    # canny edge
    low_threshold = 60#60
    high_threshold = 70# 70

    # edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # edges_img1 = cv2.Canny(np.uint8(binary_img), low_threshold, high_threshold)
    edges_img2 = cv2.Canny(np.uint8(binary_img1), low_threshold, high_threshold)
    # edges_img3 = cv2.Canny(np.uint8(img_mask), low_threshold, high_threshold)
    edges_img3 = cv2.Canny(np.uint8(img_mask2), low_threshold, high_threshold)


    # cv2.imshow("edges_img1",edges_img3)

    # warper
    # img = warper.warp(edges_img)
    # bird = warper.warp(edges_img1)
    bird1 = warper.warp(edges_img2)
    bird2 = warper.warp(edges_img3)

    # stop_line
    # 미검출시[-1, -1, -1, -1] 검출시[x1, y1, x2, y2]
    stop_line_array = stop_line(bird1)
    # print(stop_line_array)



    # cv2.imshow("bird",bird2)

    # img1, x_location1 = slidewindow.slidewindow(img)
    # img2, x_location2 = slidewindow.slidewindow(bird)
    img3, x_location3, steer = slidewindow.slidewindow(bird2)

    cv2.rectangle(img3, (180, 320),  (180+440, 320+80), (0, 0, 255), 4)

    # 정지선 그리기
    if(stop_line_array[0] != -1):

        font = cv2.FONT_HERSHEY_COMPLEX  # normal size serif font
        fontScale = 1.2
        cv2.putText(img3, 'stop!!!!!!', (10, 80), font, fontScale, (0, 0, 255), 4)

        cv2.line(img3,(stop_line_array[0], stop_line_array[1]) \
        ,(stop_line_array[2], stop_line_array[3]),(0,0,255),10)

    # print(x_location1)

    return img3, x_location3, steer


if __name__ == '__main__':
    main()
