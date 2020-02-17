#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)

  def callback(self,data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of the color we look for in the HSV space
    rlower = np.array([170,70,50])
    rupper = np.array([180,255,255])     

    # rlower = np.array([0,100,100])
    # rupper = np.array([10,255,255])     
  

    # Threshold the HSV image to get only the pixels in ranage
    onlyr = cv2.inRange(hsv, rlower, rupper)
    mask = onlyr

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, onlyr)

    redcount = np.count_nonzero(mask)
    rospy.loginfo(redcount)

    if redcount > 7500:
        rospy.loginfo('Sign Detected')

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('node3', anonymous=True)

  ic = image_converter()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)