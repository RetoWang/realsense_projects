import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


class ShapeAnalysis:

    def analysis(self, frame):
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 46])
        # change to hsv model
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # get mask
        binary = cv.inRange(hsv, lower_black, upper_black)
        # downSampled = cv.pyrDown(binary)
        # h, w = downSampled.shape
        dist = cv.distanceTransform(binary, cv.DIST_L1, cv.DIST_MASK_PRECISE)
        cv.namedWindow("distance", cv.WINDOW_NORMAL)
        cv.resizeWindow('distance', 800, 1100)
        cv.imshow("distance", dist)
        out = cv.GaussianBlur(binary, (9, 9), 0)
        # cv.imwrite('out.jpg', out)
        # cv.namedWindow("blur", cv.WINDOW_NORMAL)
        # cv.resizeWindow('blur', 800, 1100)
        # cv.imshow('blur', out)
        # 二值化图像
        print("start to detect lines...\n")
        contours, hierarchy = cv.findContours(out, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        print(len(contours))
        index = 0
        for i in range(len(contours)):
            if (len(contours[i]) > index):
                index = i
        # cv.drawContours(frame, contours, index, (0, 0, 255), 1)
        # cv.namedWindow("extract", cv.WINDOW_NORMAL)
        # cv.resizeWindow('extract', 800, 1100)
        # cv.imshow("extract", frame)
        return contours

    def search(self, a):
        length = len(a)
        m = 0
        for i in range(length):
            temp = len(a[i])
            if temp > m:
                m = i
        print(len(a[m]))
        return a[m]

    def generate(self, a):
        a = a.reshape(-1, 2)
        return a

    def curve(self, a, frame):
        h, w, _ = frame.shape
        x = a[:, 0]
        y = a[:, 1]
        # for i in range(len(x)):
        #     cv.circle(image, (x[i], y[i]), 3, (255, 0, 0), -1, 8, 0)
        #     # cv.imwrite("D:/curve.png", image)
        #     cv.namedWindow("curve", cv.WINDOW_NORMAL)
        #     cv.resizeWindow('curve', 800, 1100)
        #     cv.imshow("curve", image)
        poly = np.poly1d(np.polyfit(x, y, 3))
        print(poly)
        yvals = poly(x)
        yvals = list(yvals)
        for i in range(len(yvals)):
            yvals[i] = int(yvals[i])

        # 绘图
        # plot1 = plt.plot(x, y, 's', label='original values')
        # plot2 = plt.plot(x, yvals, 'r', label='polyfit values')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.xlim(0, yaxis)
        # plt.ylim(0, xaxis)
        # plt.legend(loc=4)  # 指定legend的位置右下角
        # plt.title('curve_fit')
        # plt.show()
        for point in zip(x, yvals):
            cv.circle(frame, point, 2, (0, 0, 255), 2)
        cv.namedWindow("image_result", cv.WINDOW_NORMAL)
        cv.resizeWindow('image_result', 800, 1100)
        cv.imshow("image_result", frame)
        return 0

    def func(x, a, b, c):
        return a * np.exp(-b * x) + c


if __name__ == "__main__":
    src = cv.imread("Photo_0621_3a.jpg")
    ld = ShapeAnalysis()
    tuple = ld.analysis(src)
    array = ld.search(tuple)
    points = ld.generate(array)
    ld.curve(points, src)
    while True:
        key = cv.waitKey(1)
        if key == ord("q"):
            break

    # cap = cv.VideoCapture(0)
    # if not cap.isOpened():
    #     print("Cannot open camera")
    #     exit()
    # while True:
    #     # Capture frame-by-frame
    #     ret, frame = cap.read()
    #     # if frame is read correctly ret is True
    #     if not ret:
    #         print("Can't receive frame (stream end?). Exiting ...")
    #         break
    #     # Our operations on the frame come here
    #     ld = ShapeAnalysis()
    #     h, w, _ = frame.shape
    #     ld = ShapeAnalysis()
    #     tuple = ld.analysis(frame)
    #     array = ld.search(tuple)
    #     points = ld.generate(array)
    #     ld.curve(points, frame)
    #     key = cv.waitKey(1)
    #     if key == ord('q'):
    #         break
    # # When everything done, release the capture
    # cap.release()
    # cv.destroyAllWindows()
