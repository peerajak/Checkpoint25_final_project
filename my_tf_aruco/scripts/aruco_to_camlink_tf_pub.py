#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library
from numpy.linalg import inv

class ArucoPoseEstimator():

    def __init__(self):
        pass

    def euler_from_quaternion(self, x, y, z, w):
        pass

    def camera_coordinate_to_pixel_position(self,focal_x,focal_y,pixel_position_principal_point_x, \
            pixel_position_principal_point_y, P_camra_coordinate):
        pass

    def detect_pose_return_tf(self):
        pass

class ArucoToCamlinkTF(Node):

    def __init__(self, aruco_frame="aruco_frame"):
        super().__init__('aruco_to_camlink_tf_node')

        self._aruco_frame = aruco_frame
        self._aruco_pose_estimator = ArucoPoseEstimator()
        
        # Create a new `TransformStamped` object.
        # A `TransformStamped` object is a ROS message that represents a transformation between two frames.
        self.transform_stamped = TransformStamped()
        # This line sets the `header.frame_id` attribute of the `TransformStamped` object.
        # The `header.frame_id` attribute specifies the frame in which the transformation is defined.
        # In this case, the transformation is defined in the `world` frame.
        self.transform_stamped.header.frame_id = "wrist_rgbd_camera_link"
        # This line sets the `child_frame_id` attribute of the `TransformStamped` object.
        # The `child_frame_id` attribute specifies the frame that is being transformed to.
        # In this case, the robot's base frame is being transformed to the `world` frame.
        self.transform_stamped.child_frame_id = self._aruco_frame

        self.Timer = self.create_timer(
            1.0, self.timer_callback
        )
        # This line creates a new `TransformBroadcaster` object.
        # A `TransformBroadcaster` object is a ROS node that publishes TF messages.
        self.br = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("aruco_to_camlink_tf_node ready!")
        

    def timer_callback(self):
        self.broadcast_new_tf()

    def broadcast_new_tf(self):
        """
        This function broadcasts a new TF message to the TF network.
        """

        # Get the current odometry data.
        #position = self.cam_bot_odom.pose.pose.position
        #orientation = self.cam_bot_odom.pose.pose.orientation

        # Set the timestamp of the TF message.
        # The timestamp of the TF message is set to the current time.
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

        # Set the translation of the TF message.
        # The translation of the TF message is set to the current position of the robot.
        self.transform_stamped.transform.translation.x = 0.1 #position.x
        self.transform_stamped.transform.translation.y = 0.1 #position.y
        self.transform_stamped.transform.translation.z = 0.1 #position.z

        # Set the rotation of the TF message.
        # The rotation of the TF message is set to the current orientation of the robot.
        self.transform_stamped.transform.rotation.x = 0.1 #orientation.x
        self.transform_stamped.transform.rotation.y = 0.2 #orientation.y
        self.transform_stamped.transform.rotation.z = 0.3 #orientation.z
        self.transform_stamped.transform.rotation.w = 0.4 #orientation.w

        # Send (broadcast) the TF message.
        self.br.sendTransform(self.transform_stamped)


def main(args=None):

    rclpy.init()
    aruco_to_cam_tf_obj = ArucoToCamlinkTF()
    rclpy.spin(aruco_to_cam_tf_obj)

if __name__ == '__main__':
    main()