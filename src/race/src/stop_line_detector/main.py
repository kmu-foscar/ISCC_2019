import cv2
import numpy as np

# Red Color Range
HSV_RED_LOWER = np.array([0, 100, 100])
HSV_RED_UPPER = np.array([10, 255, 255])
HSV_RED_LOWER1 = np.array([140, 50, 80])
HSV_RED_UPPER1 = np.array([179, 255, 255])

HSV_YELLOW_LOWER = np.array([10, 80, 120])
HSV_YELLOW_UPPER = np.array([40, 255, 255])

RGB_WHITE_LOWER = np.array([100, 100, 190]);
RGB_WHITE_UPPER = np.array([255, 255, 255]);

cap = cv2.VideoCapture("TEST14.avi");

while True:
    ret, img = cap.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # red
    redBinary = cv2.inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER)
    redBinary1 = cv2.inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1)
    redBinary = cv2.bitwise_or(redBinary, redBinary1)

	# 2. yellow
    yellowBinary = cv2.inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER)

    # write
    # whiteBinary = cv2.inRange(img, RGB_WHITE_LOWER, RGB_WHITE_UPPER)
    ret, whiteBinary = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3, 3), np.uint8)

    redBinary = cv2.dilate(redBinary, kernel, iterations=1)
    redBinary = cv2.dilate(redBinary, kernel, iterations=1)
    redBinary = cv2.dilate(redBinary, kernel, iterations=1)
    redBinary = cv2.dilate(redBinary, kernel, iterations=1)
    redBinary = cv2.dilate(redBinary, kernel, iterations=1)

    yellowBinary = cv2.dilate(yellowBinary, kernel, iterations=1)
    yellowBinary = cv2.dilate(yellowBinary, kernel, iterations=1)
    yellowBinary = cv2.dilate(yellowBinary, kernel, iterations=1)
    yellowBinary = cv2.dilate(yellowBinary, kernel, iterations=1)
    yellowBinary = cv2.dilate(yellowBinary, kernel, iterations=1)

    whiteBinary = cv2.dilate(whiteBinary, kernel, iterations=1)
    whiteBinary = cv2.dilate(whiteBinary, kernel, iterations=1)


    # red delete
    result1 = cv2.bitwise_and(gray, ~redBinary)

    # yellow delete
    result2 = cv2.bitwise_and(gray, ~yellowBinary)

    # white alone
    result3 = cv2.bitwise_and(gray, whiteBinary)



    result4 = cv2.bitwise_and(cv2.bitwise_and(result1, result2), result3)

    edges = cv2.Canny(result4, 120, 270)

    cv2.imshow("result4", edges)

    lines = cv2.HoughLines(edges,1,np.pi/180,100)
    for i in range(len(lines)):
        for rho, theta in lines[i]:
            tempTheta = theta/np.pi *180
            if(tempTheta > 88 and tempTheta < 92):
                print("hi 5")
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0+1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 -1000*(a))
                cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

            # if(tempTheta < 180 and tempTheta > 175):
            #     print("hi 175")
            #     a = np.cos(theta)
            #     b = np.sin(theta)
            #     x0 = a*rho
            #     y0 = b*rho
            #     x1 = int(x0 + 1000*(-b))
            #     y1 = int(y0+1000*(a))
            #     x2 = int(x0 - 1000*(-b))
            #     y2 = int(y0 -1000*(a))
            #
            #     cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
            #

    cv2.imshow("img", img)


    # cv2.waitKey(0)
    if cv2.waitKey(1) == 27:
        break;
