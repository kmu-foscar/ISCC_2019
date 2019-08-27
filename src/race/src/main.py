#!/usr/bin/env python
import rospy
import cv2
import threading
# import Queue as que
import time
import numpy as np

# import roslib
import sys
# import rospy

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

import os

warper = Warper()
slidewindow  = SlideWindow()
pidcal = PidCal()

cv_image = None
ack_publisher = None
car_run_speed = 0.5

lane_info_pub = rospy.Publisher('lane_info', lane_info, queue_size=1)
drive_values_pub = rospy.Publisher('drive_values', drive_values, queue_size=1)

def auto_drive(pid):
    global car_run_speed
    w = 0
    if -0.065 < pid and pid < 0.065:
        w = 1
    else:
        w = 0.3

    if car_run_speed < 5.0 * w:
        car_run_speed += 0.002 * 10
    else:
        car_run_speed -= 0.003 * 10

    # ack_msg = AckermannDriveStamped()
    # ack_msg.header.stamp = rospy.Time.now()
    # ack_msg.header.frame_id = ''
    # ack_msg.drive.steering_angle = pid
    # ack_msg.drive.speed = car_run_speed
    # ack_publisher.publish(ack_msg)

    drive_value = drive_values()
    drive_value.throttle = car_run_speed;
    drive_value.steering = -pid/0.074

    print('steer: ', drive_value.steering)
    print('speed: ', car_run_speed)

def main():
    rospy.init_node('lane_detector_goni_node')
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("TEST14.avi")

    while True:
        ret, img = cap.read()
        img1, x_location = process_image(img)
        cv2.imshow('result', img1)
        if x_location != None:
            pid = round(pidcal.pid_control(int(x_location)), 6)
            print(pid)
            auto_drive(pid)
            # print pid
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

      #out.write(img1)
      #out2.write(cv_image)


def process_image(frame):

    # grayscle
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # canny edge
    low_threshold = 60#60
    high_threshold = 70# 70
    edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # warper
    img = warper.warp(edges_img)
    img1, x_location = slidewindow.slidewindow(img)

    return img1, x_location

if __name__ == '__main__':
    main()
