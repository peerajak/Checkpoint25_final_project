#! /usr/bin/env python3

from __future__ import print_function

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np
from cv_bridge import CvBridge

class image_converter(Node):
  def __init__(self):
    super().__init__('bgr_publisher')
    self.subscription = self.create_subscription(
            Image,
            '/wrist_rgbd_depth_sensor/image_raw',  
            self.image_callback, 10)
    self.publisher = self.create_publisher(Image, '/wrist_rgbd_depth_sensor/image_bgr', 10)
    self.cv_bridge = CvBridge()



  def image_callback(self, msg: Image) -> None:
    try:
      cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        self.get_logger().error("Error converting ROS Image to OpenCV format: {0}".format(e))
        return

    (rows,cols,channels) = cv_image.shape
    if cols > 160 and rows > 160 :
      cv2.circle(cv_image, (80,80), 50, (255,255,255))

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv_image))
    except Exception as e:
      print(e)

def main(args):
    rclpy.init(args=args)
    img_convert_publisher = image_converter()
    rclpy.spin(img_convert_publisher)
    img_convert_publisher.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main(sys.argv)