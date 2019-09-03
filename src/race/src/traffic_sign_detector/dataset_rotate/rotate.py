import cv2

src = cv2.imread("static.png", cv2.IMREAD_COLOR)

height, width, channel = src.shape

while True:
    matrix = cv2.getRotationMatrix2D((width/2, height/2), 3, 1)
    dst1 = cv2.warpAffine(src, matrix, (width, height))

    cv2.imshow("dst1", dst1)

    matrix = cv2.getRotationMatrix2D((width/2, height/2), -3, 1)
    dst2 = cv2.warpAffine(src, matrix, (width, height))

    cv2.imshow("dst2", dst2)
    matrix = cv2.getRotationMatrix2D((width/2, height/2), 5, 1)
    dst3 = cv2.warpAffine(src, matrix, (width, height))

    cv2.imshow("dst3", dst3)

    matrix = cv2.getRotationMatrix2D((width/2, height/2), -5, 1)
    dst4 = cv2.warpAffine(src, matrix, (width, height))

    cv2.imshow("dst4", dst4)


    cv2.waitKey(0)
    cv2.destroyAllWindows()
