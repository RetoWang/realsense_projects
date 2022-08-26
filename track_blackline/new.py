import numpy as np
import cv2


class linedetector:
    def __init__(self):
        self.lines = []

    def find_lines(self, frame):

        # set HSV interval of black
        lower_black = np.array([5, 25, 150])
        upper_black = np.array([170, 166, 255])

        # change to hsv model
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # select black pixel and generate binary image
        binary = cv2.inRange(hsv, lower_black, upper_black)
        cv2.namedWindow("binary image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow('binary image', 800, 1100)
        cv2.imshow("binary image", binary)

        # downsample the image to increase running speed
        # downSampled = cv2.pyrDown(binary)
        # dist = downSampled
        # cv2.namedWindow("downSampled image", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('downSampled image', 800,1100)
        # cv2.imshow("downSampled image", dist)

        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(binary, contours, 2, (60, 100, 100), -1)
        # cv2.namedWindow("Contours", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Contours', 800,1100)
        # cv2.imshow("Contours", binary)

        temp = 0
        index = 0
        for i in range(len(contours) - 1):
            Area = cv2.contourArea(contours[i])
            if Area > temp:
                temp = Area
                index = i

        mask = cv2.drawContours(frame, contours, index, (0, 0, 255), 1)
        cv2.namedWindow("Largest Contours", cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Largest Contours', 800, 1100)
        cv2.imshow("Largest Contours", mask)

        temp = 0
        hierarchy = np.squeeze(hierarchy)
        if hierarchy[index][2] != -1:
            son_index = hierarchy[index][2]
            temp = cv2.contourArea(contours[son_index])
            while (hierarchy[son_index][0] != -1):
                son_index = hierarchy[son_index][0]
                Area = cv2.contourArea(contours[son_index])
                if Area > temp:
                    temp = Area
                    largest_son_index = son_index

            Pic_Line = cv2.drawContours(frame, contours, largest_son_index, (255, 0, 0), 1)
            cv2.namedWindow("Line", cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Line', 800, 1100)
            cv2.imshow("Line", Pic_Line)

        target = np.squeeze(contours[index])
        print(target.shape)
        print(target)

        xpts = target[:][0]
        ypts = target[:][1]

        # curve fitting (5th power polynomial)
        coeffi = 0
        try:
            coeffi = np.polyfit(ypts, xpts, 4)
        except:
            pass
        p1 = np.poly1d(coeffi)
        xvals = p1(ypts)
        xvals = list(xvals)
        for i in range(len(xvals)):
            xvals[i] = int(xvals[i])
        h, w = binary.shape
        # Coordinateds converted to opencv format
        for i in range(len(xvals)):
            xvals[i] = h - xvals[i]

        # show fit curve to original image
        for point in zip(ypts, xvals):
            cv2.circle(frame, point, 2, (0, 0, 255), 2)
        cv2.namedWindow("image_result", cv2.WINDOW_NORMAL)
        cv2.resizeWindow('image_result', 800, 1100)
        cv2.imshow("image_result", frame)

        return frame


if __name__ == "__main__":
    image = cv2.imread("Photo_0621_3a.jpg")
    ld = linedetector()
    lines = ld.find_lines(image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()