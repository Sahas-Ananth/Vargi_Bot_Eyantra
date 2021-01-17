#!/usr/bin/env python
# This ia an alternate version of the code which can be used for color detection as specified in task-4
import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  def get_dominant_colour(self, img):
      #converting image from BGR to hue saturation value
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #lower and upper threshold value of yellow
        ly=np.array([25,70,120])
        uy=np.array([30,255,255])
        #lower and upper threshold value of green
        lg=np.array([40,70,80])
        ug=np.array([70,255,255])
        #lower and upper threshold value of red
        lr=np.array([0,50,120])
        ur=np.array([10,255,255])
        #creating masks,contours for green ,red,yellow
        mask1=cv2.inRange(hsv,ly,uy)
        mask2=cv2.inRange(hsv,lg,ug)
        mask3=cv2.inRange(hsv,lr,ur)
        cnts1=cv2.findContours(mask1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnts1=imutils.grab_contours(cnts1)
        cnts2=cv2.findContours(mask2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnts2=imutils.grab_contours(cnts2)
        cnts3=cv2.findContours(mask3,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cnts3=imutils.grab_contours(cnts3)
        for c in cnts1:
                area1=cv2.contourArea(c)
                if area1:
                        return "Yellow"
                        break
        for c in cnts2:
                area2=cv2.contourArea(c)
                if area2:
                        return "Green"
                        break
        for c in cnts3:
                area3=cv2.contourArea(c)
                if area3:
                        return "Red"
                        break
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(image, (720/2, 1280/2)) 

    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

    rospy.loginfo(self.get_dominant_colour(image))

    cv2.waitKey(3)


def main(args):
  
  rospy.init_node('node_eg2_colour_detection', anonymous=True)

  ic = Camera1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
