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
        gamma = 0.5

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

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        cv_image = cv2.resize(cv_image, (600, 1000))

        rospy.loginfo('Increasing brightness')
        cv_image = self.increase_brightness(cv_image)
        cv2.imshow('gamma correction', cv_image)

        rospy.loginfo('Decoding image')
        boxes, grps = self.get_qr_data(cv_image)

        # NOTE: Assumes all packages found.
        for i, row in enumerate(grps):
            for j, col in enumerate(row):
                self.packages['package{}{}'.format(i, j)] = col.data

        for (x, y, w, h, col) in boxes:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 0), 5)

        self.image_sub.unregister()
        cv2.imshow('Image', cv_image)
        print (self.packages)
        cv2.waitKey(0)


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
