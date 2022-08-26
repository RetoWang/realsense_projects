import cv2 as cv
import numpy as np

from scipy.optimize import curve_fit


class ShapeAnalysis:

    def analysis(self, frame):
        lower = np.array([0, 65, 148])
        upper = np.array([18, 255, 255])
        # change to hsv model
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # get mask
        binary = cv.inRange(hsv, lower, upper)
        dist = cv.distanceTransform(binary, cv.DIST_L1, cv.DIST_MASK_PRECISE)
        cv.namedWindow("distance", cv.WINDOW_NORMAL)
        # cv.resizeWindow('distance', 800, 1100)
        cv.imshow("distance", dist)
        binary = cv.GaussianBlur(binary, (9, 9), 0)
        # 二值化图像
        print("start to detect lines...\n")
        contours, hierarchy = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        print(len(contours))
        index = 0
        temp = 0
        for i in range(len(contours)):
            # 计算轮廓所包含的面积
            area = cv.contourArea(contours[i])
            # 计算轮廓周长
            # length = cv.arcLength(contours[i], True)
            if area > temp:
                temp = area
                index = i

        cv.drawContours(frame, contours, index, (0, 0, 255), 1)
        cv.namedWindow("extract", cv.WINDOW_NORMAL)
        # cv.resizeWindow('extract', 800, 1100)
        cv.imshow("extract", frame)

        final_idx = 0
        hierarchy = np.squeeze(hierarchy)
        print(hierarchy.shape)
        if hierarchy[index][2] != -1:
            index = hierarchy[index][2]
            temp = cv.contourArea(contours[index])
            while hierarchy[index][0] != -1:
                index = hierarchy[index][0]
                area = cv.contourArea(contours[index])
                if area > temp:
                    temp = area
                    final_idx = index

        cv.drawContours(frame, contours, final_idx, (255, 0, 0), 1)
        cv.namedWindow("Line", cv.WINDOW_NORMAL)
        # cv.resizeWindow('Line', 800, 1100)
        cv.imshow("Line", frame)

        a = contours[final_idx].reshape(-1, 2)
        return a

    def curve(self, a, frame):
        h, w, _ = frame.shape
        x = a[:, 0]
        y = a[:, 1]

        poly = np.poly1d(np.polyfit(x, y, 3))
        print(poly)
        yvals = poly(x)
        yvals = list(yvals)
        for i in range(len(yvals)):
            yvals[i] = int(yvals[i])

        points = zip(x, yvals)
        points = points.reshape((-1, 1, 2))
        cv.polylines(frame, [points], True, (0, 0, 255))

        # for point in zip(x, yvals):
        #     cv.circle(frame, point, 2, (0, 0, 255), 2)
        cv.namedWindow("image_result", cv.WINDOW_NORMAL)
        # cv.resizeWindow('image_result', 800, 1100)
        cv.imshow("image_result", frame)
        return 0

    def func(x, a, b, c):
        return a * np.exp(-b * x) + c


if __name__ == "__main__":
    # src = cv.imread("Photo_0621_3a.jpg")
    # ld = ShapeAnalysis()
    # array = ld.analysis(src)
    # ld.curve(array, src)
    # while True:
    #     key = cv.waitKey(1)
    #     if key == ord("q"):
    #         break

    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        try:
            ld = ShapeAnalysis()
            h, w, _ = frame.shape
            ld = ShapeAnalysis()
            array = ld.analysis(frame)
            ld.curve(array, frame)
            key = cv.waitKey(1)
            if key == ord('q'):
                break
        except:
            pass
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()
