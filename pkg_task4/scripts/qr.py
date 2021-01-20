#!/usr/bin/env python

import rospy
import cv2
import copy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from pyzbar.pyzbar import decode


def sort_by_x(e):
    return e.rect[0]


def sort_by_y(e):
    return e.rect[1]


class Camera1:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/eyrc/vb/camera_1/image_raw", Image, self.callback)
        self.packages = {}

    def sharpen_image(self, img):
        filt = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        sharpeningKernel = np.array(
            ([0, -1, 0], [-1, 5, -1], [0, -1, 0]), dtype="int")
        sharpen_img_1 = cv2.filter2D(img, -1, filt)
        cv2.imshow("sharpned image", sharpen_img_1)
        return sharpen_img_1

    def increase_brightness(self, image):
        alpha = 2.2
        beta = 50

        new_image = np.zeros(image.shape, image.dtype)
        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                for c in range(image.shape[2]):
                    new_image[y, x, c] = np.clip(
                        alpha*image[y, x, c] + beta, 0, 255)

        return new_image

    def increase_brightness2(self, image):
        """
            Best gamma value for res of 600 x 1000 is 0.5
        """
        gamma = 0.9

        lookUpTable = np.empty((1, 256), np.uint8)
        for i in range(256):
            lookUpTable[0, i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
        res = cv2.LUT(image, lookUpTable)
        return res

    def _get_borders(self, qr_result):
        boxes = []
        for qr in qr_result:
            (x, y, w, h) = qr.rect
            boxes.append((x, y, w, h, qr.data))
        return boxes

    def get_qr_data(self, arg_image):
        qr_result = decode(arg_image)

        grps = []
        if (len(qr_result) > 0):
            # NOTE: The below method to group the packages by row and col assumes
            # all packages has been detected. In case any packages goes missing
            # the below method will not give the correct grouping.
            qr_result.sort(key=sort_by_y)

            i = 1
            row = []
            for qr in qr_result:
                row.append(qr)
                if i % 3 == 0:
                    i = 0
                    row.sort(key=sort_by_x)
                    grps.append(row)
                    row = []

                i += 1

        boxes = self._get_borders(qr_result)
        return (boxes, grps)

    def sort_by_row_and_col(self, boxes):
        """ box is a array of tuples (x, y) """
        err = 2
        n = len(boxes)
        res = []

        i = 0
        while i < n:
            row = []
            xi, yi, _ = boxes[i]
            row.append(boxes[i])
            j = i + 1
            while j < n:
                xj, yj, _ = boxes[j]
                if abs(yi - yj) < err:
                    row.append(boxes[j])
                else:
                    i = j
                    break
                j += 1
            row.sort(key=lambda tup: tup[0])
            res.append(row)
            if j >= n:
                break

        return res

    def find_missing(self, boxes, err):
        cols = [105, 263, 419]

        xs = [xi for xi, _, _ in boxes]

        missing = []
        for i, col in enumerate(cols):
            found = False
            for x in xs:
                if abs(x - col) < err:
                    found = True
            if not found:
                missing.append(i)
        return missing

    def name_boxes(self, boxes):
        rows = []
        for (x, y, w, h, col) in boxes:
            rows.append((x, y, col))

        err = 7

        rows = self.sort_by_row_and_col(rows)

        res = []
        for i, row in enumerate(rows):
            if len(row) == 3:
                res.append(row)
                continue
            missing = self.find_missing(row, err)
            for j in missing:
                row.insert(j, None)
            res.append(row)

        return res

    def box_name_to_dict(self, rows):
        boxes = {}
        for i, row in enumerate(rows):
            for j, col in enumerate(row):
                if col is None:
                    continue
                boxes['packagen{}{}'.format(i, j)] = col[2]

        return boxes

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        cv_image = cv2.resize(cv_image, (600, 1000))

        rospy.loginfo('Increasing brightness')
        cv_image = self.increase_brightness2(cv_image)
        cv2.imshow('gamma correction', cv_image)

        rospy.loginfo('Sharpening image')
        cv_image = self.sharpen_image(cv_image)

        rospy.loginfo('Decoding image')
        boxes, grps = self.get_qr_data(cv_image)

        box_name = self.name_boxes(boxes)
        print (self.box_name_to_dict(box_name))

        for (x, y, w, h, col) in boxes:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 0), 5)

        self.image_sub.unregister()
        cv2.imshow('Image', cv_image)
        print (self.packages)
        cv2.waitKey(0)
    sharpeningKernel = np.array(
        ([0, -1, 0], [-1, 5, -1], [0, -1, 0]), dtype="int")


def main():
    rospy.init_node('node_eg3_qr_decode', anonymous=True)
    ic = Camera1()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
