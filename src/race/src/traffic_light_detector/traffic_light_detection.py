import cv2
import numpy as np

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

cap = cv2.VideoCapture("예선2.mov");

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
    grayBinary = cv2.erode(grayBinary, kernel, iterations = 3)
    grayBinary = cv2.dilate(grayBinary, kernel, iterations = 7)

    mask = cv2.cvtColor(grayBinary, cv2.COLOR_GRAY2BGR)
    cv2.imshow("mask", mask)

    mask = cv2.bitwise_and(frame, mask)



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
            print("멈춰씨발년아")
        elif redAndYellowPixelCnt and greenPixelCnt :
            print("좌회전 조져시발년")

    else :
        print("멈추지마 씨발럼아 노빠꾸다")


    cv2.imshow("sibar", frame)

    cv2.waitKey(0)
    # if cv2.waitKey(1) == 27:
        # break;
