#!/usr/bin/env python

import rospy

import cv2
import numpy as np
from std_msgs.msg import UInt8;


# Red Color Range
HSV_RED_LOWER = np.array([0, 100, 100])
HSV_RED_UPPER = np.array([10, 255, 255])
HSV_RED_LOWER1 = np.array([160, 100, 100])
HSV_RED_UPPER1 = np.array([190, 255, 255])

# Yellow Color Range
HSV_YELLOW_LOWER = np.array([0, 80, 120])
HSV_YELLOW_UPPER = np.array([30, 255, 255])

# Green Color Range
HSV_GREEN_LOWER = np.array([50, 130, 65])
HSV_GREEN_UPPER = np.array([120, 255, 255])

cap = cv2.VideoCapture("yellow.mov");

# 연속으로 인식해야 메세지 쏴준다.
ACCURACY_COUNT = 10

zeroModeCnt = 0
oneModeCnt = 0
twoModeCnt = 0

rospy.init_node('traffic_light_node')
trafficLightPub = rospy.Publisher('traffic_light', UInt8, queue_size=1)

while True:
    ret, frame = cap.read()

    #resize
    width = 640
    height = 480
    frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)

    frame = frame[0:400, 200:540]

	#gray and hsv transform
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("gray", gray)
    cv2.imshow("hsv", hsv)

    #gray로 이진화(신호등 영역을 검출하기 위함)
    ret, grayBinary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

    #팽창
    kernel = np.ones((3, 3), np.uint8)
    grayBinary = cv2.erode(grayBinary, kernel, iterations = 2)
    grayBinary = cv2.dilate(grayBinary, kernel, iterations = 9)

    mask = cv2.cvtColor(grayBinary, cv2.COLOR_GRAY2BGR)
    cv2.imshow("mask", mask)

    mask = cv2.bitwise_and(frame, mask)

    cv2.imshow("magic", mask)



    #find contours
    image, contours, hierachy = cv2.findContours(grayBinary, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    maxSize = -1
    maxRedAndYellowCnt = 0

    goodRoiPosition = [0, 0, 0, 0]
    findGoodRoiCheck = False


    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < 4000 and area > 10.0 :
            x, y, w, h = cv2.boundingRect(cnt)
            rate = w / h

            if rate > 0.9:
                # binary
                # 1. red
                inputImage = hsv[y:y+h, x:x+w]
                redBinary = cv2.inRange(inputImage, HSV_RED_LOWER, HSV_RED_UPPER)
                redBinary1 = cv2.inRange(inputImage, HSV_RED_LOWER1, HSV_RED_UPPER1)
                redBinary = cv2.bitwise_or(redBinary, redBinary1)

                # 2. yellow
                yellowBinary = cv2.inRange(inputImage, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER)

                # 1 or 2
                redAndYellowBinary = cv2.bitwise_or(redBinary, yellowBinary)

                # 3. green
                greenBinary = cv2.inRange(inputImage, HSV_GREEN_LOWER, HSV_GREEN_UPPER)

                redAndYellowPixelCnt = cv2.countNonZero(redAndYellowBinary)
                greenPixelCnt = cv2.countNonZero(greenBinary)

                # print(redAndYellowPixelCnt, "sibar", greenPixelCnt)

                if redAndYellowPixelCnt > maxRedAndYellowCnt :
                    # print("RedCnt : ", redAndYellowPixelCnt)
                    # print("Position : ", x, " ", y, " ", w, " ", h)

                    goodRoiPosition = [x, y, w, h]
                    maxRedAndYellowCnt = redAndYellowPixelCnt
                    findGoodRoiCheck = True;



    x, y, w, h = goodRoiPosition
    # print("roi 좌표: ", y, "-", x)

    if findGoodRoiCheck :
        goodRoiImg = hsv[y:y+h, x:x+w]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (200, 2, 50), 2)


        # binary
        # 1. red
        redBinary = cv2.inRange(goodRoiImg, HSV_RED_LOWER, HSV_RED_UPPER)
        redBinary1 = cv2.inRange(goodRoiImg, HSV_RED_LOWER1, HSV_RED_UPPER1)
        redBinary = cv2.bitwise_or(redBinary, redBinary1)

        # 2. yellow
        yellowBinary = cv2.inRange(goodRoiImg, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER)

        # 1 or 2
        redAndYellowBinary = cv2.bitwise_or(redBinary, yellowBinary)

        cv2.imshow("redAndYellow", redAndYellowBinary)

        # 3. green
        greenBinary = cv2.inRange(goodRoiImg, HSV_GREEN_LOWER, HSV_GREEN_UPPER)

        cv2.imshow("green", greenBinary)
        redAndYellowPixelCnt = cv2.countNonZero(redAndYellowBinary)
        greenPixelCnt = cv2.countNonZero(greenBinary)

        # print(redAndYellowPixelCnt, " - ", greenPixelCnt)


        if redAndYellowPixelCnt and not greenPixelCnt :
            zeroModeCnt = zeroModeCnt + 1
            oneModeCnt = 0
            twoModeCnt = 0
        elif redAndYellowPixelCnt and greenPixelCnt :
            zeroModeCnt = 0
            oneModeCnt = oneModeCnt + 1
            twoModeCnt = 0

    else :
        zeroModeCnt = 0
        oneModeCnt = 0
        twoModeCnt = twoModeCnt + 1


    msg = UInt8()
    if zeroModeCnt >= ACCURACY_COUNT :
        zeroModeCnt = 0
        oneModeCnt = 0
        twoModeCnt = 0
        msg.data = 0
        trafficLightPub.publish(msg)
        print("멈춰")
    elif oneModeCnt >= ACCURACY_COUNT :
        zeroModeCnt = 0
        oneModeCnt = 0
        twoModeCnt = 0
        msg.data = 1
        trafficLightPub.publish(msg)
        print("좌회전")
    elif twoModeCnt >= ACCURACY_COUNT:
        zeroModeCnt = 0
        oneModeCnt = 0
        twoModeCnt = 0
        msg.data = 2
        trafficLightPub.publish(msg)
        print("멈추지마라")


    cv2.imshow("frame", frame)

    cv2.waitKey(0)
    # if cv2.waitKey(1) == 27:
        # break;
