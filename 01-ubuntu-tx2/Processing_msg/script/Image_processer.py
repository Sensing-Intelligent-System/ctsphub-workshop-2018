#!/usr/bin/env python

import sys

# numpy
import numpy as np

# OpenCV
import cv2

# Birdge OpenCV and ROS
from cv_bridge import CvBridge, CvBridgeError

# Ros libraries
import roslib
import rospy

# Ros Messages
import sensor_msgs.msg

# Synchronize RGB and Depth message
from message_filters import ApproximateTimeSynchronizer, Subscriber

# Image processing function
import Image_module as IM



class image_topic_reciever:

  def __init__(self):
    print "init class"
    self.bridge = CvBridge()
    self.image_sub = Subscriber("/camera/rgb/image_rect_color", sensor_msgs.msg.Image)
    self.camera_sub = Subscriber("/camera/depth_registered/sw_registered/image_rect", sensor_msgs.msg.Image)
    self.ats = ApproximateTimeSynchronizer([self.image_sub, self.camera_sub], queue_size=5, slop=0.1)
    self.ats.registerCallback(self.gotimage)
    self.image_pub = rospy.Publisher("/desired_rgb",sensor_msgs.msg.Image, queue_size=5)

  def gotimage(self, RGB, Depth):
    assert RGB.header.stamp == Depth.header.stamp

    try:
      cv_RGB = self.bridge.imgmsg_to_cv2(RGB, "bgr8")
      cv_Depth = self.bridge.imgmsg_to_cv2(Depth, "32FC1")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_RGB.shape

    desired_RGB = self.cv_visualize(cv_RGB, cv_Depth)
    #self.pub_desired_rgb(desired_RGB)

  def cv_visualize(self, cv_RGB, cv_Depth):
    valid_RGB = IM.image_in_workrange(cv_RGB,cv_Depth)
    desired_RGB = IM.search_desired_pixel(cv_RGB,cv_Depth)
    cv2.imshow('0. RGB', cv_RGB)
    cv2.imshow('1. Depth', cv_Depth)
    #cv2.imshow('2. Valid RGB', valid_RGB)
    #cv2.imshow('3. Desired Range', desired_RGB)
    cv2.waitKey(1)
    return desired_RGB

  def pub_desired_rgb(self,cv_image):
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  ic = image_topic_reciever()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
