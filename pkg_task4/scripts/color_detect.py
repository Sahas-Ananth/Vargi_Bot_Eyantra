#!/usr/bin/env python

"""
This module detects the colour of the package using the QR code
and stores this information. Which is then accessed by the
UR5 sorter node to put the boxes in the correct position.
"""
import numpy as np

from pyzbar.pyzbar import decode

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import actionlib
from sensor_msgs.msg import Image

DEBUG_SHOW_IMAGE = False

def sort_by_x(obj):
    return obj.rect[0]

def sort_by_y(obj):
    return obj.rect[1]

def show_image(title, image):
    if DEBUG_SHOW_IMAGE:
        cv2.imshow(title, image)

class Camera1(object):
    def __init__(self):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            "/eyrc/vb/camera_1/image_raw", Image, self.callback)

        self.MAX_TRY = 5
        self.GAMMA = 0.9
        self.packages = {}
        self.data_frame = None

    def sharpen_image(self, img):
        filt = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        sharpen_img_1 = cv2.filter2D(img, -1, filt)
        show_image("sharpend image", sharpen_img_1)
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

    def increase_brightness2(self, image, gamma = 0.5):
        """
            Gamma correct the image
        """
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

        qr_result.sort(key=sort_by_y)
        boxes = self._get_borders(qr_result)
        return (boxes, len(qr_result))

    def sort_by_row_and_col(self, boxes):
        """ box is a array of tuples (x, y) """
        err = 2
        n = len(boxes)
        res = []

        i = 0
        while i < n:
            row = []
            _, yi, _ = boxes[i]
            row.append(boxes[i])
            j = i + 1
            while j < n:
                _, yj, _ = boxes[j]
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
        # NOTE: The below values have to be changed when the resolution
        # of the image is changed.
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
        for (x, y, _, _, col) in boxes:
            rows.append((x, y, col))

        err = 7

        rows = self.sort_by_row_and_col(rows)

        res = []
        for row in rows:
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
        self.data_frame = data

    def detect_packages(self, goal = None):
        while self.data_frame is None:
            rospy.sleep(0.5)

        data = self.data_frame
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        cv_image = cv2.resize(cv_image, (600, 1000))

        while self.MAX_TRY > 0:
            rospy.loginfo('QRColorDetection: Gamma correcting the image')
            cv_image = self.increase_brightness2(cv_image, self.GAMMA)
            show_image('gamma correction', cv_image)

            rospy.loginfo('QRColorDetection: Sharpening image')
            cv_image = self.sharpen_image(cv_image)

            rospy.loginfo('QRColorDetection: Decoding image')
            boxes, npackages = self.get_qr_data(cv_image)
            rospy.loginfo('QRColorDetection: Found {} packages'.format(npackages))

            if npackages >= 9:
                break
            rospy.loginfo('QRColorDetection: Less than 9 packages detected trying again')

            self.MAX_TRY -= 1
            self.GAMMA -= 0.1

        if self.MAX_TRY <= 0:
            rospy.logerr('QRColorDetection: Detection of atleast 9 packages failed')

        box_name = self.name_boxes(boxes)

        self.packages = self.box_name_to_dict(box_name)
        rospy.loginfo('Detected the following packages: ' + str(self.packages))

        # NOTE: Can directly store a dictionary in param server but defaulting to store
        # string since most code has been designed to work that way.
        rospy.set_param("DetectedPackages", self.packages)
        rospy.loginfo("QRColorDetection: set param done")

        if DEBUG_SHOW_IMAGE:
            for (x, y, w, h, _) in boxes:
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 0), 5)

        show_image('Final Image', cv_image)
        if DEBUG_SHOW_IMAGE:
            cv2.waitKey(0)


def main():
    rospy.init_node('node_t4_color_detect', anonymous=True)

    # Wait for gazebo to load all packages
    rospy.sleep(10)
    Camera1().detect_packages()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    if DEBUG_SHOW_IMAGE:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
