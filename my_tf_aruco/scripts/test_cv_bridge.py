#! /usr/bin/env python3

from __future__ import print_function

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

class image_converter(Node):
  # Dictionary that was used to generate the ArUco marker
  aruco_dictionary_name = "DICT_4X4_100"
  aruco_marker_side_length = 0.026 
 
  # The different ArUco dictionaries built into the OpenCV library. 
  ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
  }

  def __init__(self):
    super().__init__('bgr_publisher')
    self.subscription = self.create_subscription(
            Image,
            '/wrist_rgbd_depth_sensor/image_raw',  
            self.image_callback, 10)
    self.publisher = self.create_publisher(Image, '/wrist_rgbd_depth_sensor/image_bgr', 10)
    self.cv_bridge = CvBridge()


  def detect_pose_return_tf(self):
    # Check that we have a valid ArUco marker
    if self.ARUCO_DICT.get(self.aruco_dictionary_name, None) is None:
        print("[INFO] ArUCo tag of is not supported")
        sys.exit(0)
    
    # Load the camera parameters from the saved file
    #cv_file = cv2.FileStorage( camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
    #mtx = cv_file.getNode('K').mat()
    #dst = cv_file.getNode('D').mat()
    #cv_file.release()
    mtx_np = np.array([ [759.895784, 0.000000, 312.753105],[0.000000, 762.113647, 214.923553], [0., 0., 1.]], np.float32)
    mtx = mtx_np
    dst_np = np.array([0.062948, -0.273568, 0.005933, -0.001056, 0.000000], np.float32)   
    prj_np = np.array([[761.265137, 0.000000, 311.720175, 0.000000],\
                        [0.000000, 764.304443, 215.883204, 0.000000],\
                        [0.000000, 0.000000, 1.000000, 0.000000]], np.float32)   
    dst = dst_np
    # Load the ArUco dictionary
    print("[INFO] detecting '{}' markers...".format(self.aruco_dictionary_name))
    this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.aruco_dictionary_name])
    this_aruco_parameters = cv2.aruco.DetectorParameters_create()
    
    # Start the video stream
    # TODO change cap to the topic image (self.cv_image)

    object_points = np.array([(-self.aruco_marker_side_length/2., self.aruco_marker_side_length/2., 0),\
                                (self.aruco_marker_side_length/2., self.aruco_marker_side_length/2., 0),\
                                (self.aruco_marker_side_length/2., -self.aruco_marker_side_length/2., 0),\
                                (-self.aruco_marker_side_length/2., -self.aruco_marker_side_length/2., 0)],\
                            dtype=np.float32).reshape(4,1,3)
    if self.cv_image is None:
            print("failed to capture videos")
            return

    
    
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.

    detectingImage = self.cv_image.copy() 

    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
        detectingImage , this_aruco_dictionary, parameters=this_aruco_parameters,
        cameraMatrix=mtx, distCoeff=dst)

    # Check that at least one ArUco marker was detected
    if marker_ids is not None: 
        num_markers = len(marker_ids)
        # print('corners ',corners)
        # print('marker_ids', marker_ids)
        # Draw a square around detected markers in the video frame
        cv2.aruco.drawDetectedMarkers(detectingImage , corners, marker_ids)
        #print(marker_ids)
        #print(corners)

        
        # Print the pose for the ArUco marker
        # The pose of the marker is with respect to the camera lens frame.
        # Imagine you are looking through the camera viewfinder, 
        # the camera lens frame's:
        # x-axis points to the right
        # y-axis points straight down towards your toes
        # z-axis points straight ahead away from your eye, out of the camera
        for i, marker_id in enumerate(marker_ids):
            realign_corners = np.zeros((4,2), dtype =np.float32)
            realign_corners[0] = corners[i][:,0,:].flatten()
            realign_corners[1] = corners[i][:,1,:].flatten()
            realign_corners[2] = corners[i][:,2,:].flatten()
            realign_corners[3] = corners[i][:,3,:].flatten()
            print(realign_corners)  
            image_points = realign_corners.reshape(4,1,2)
            #print(image_points)
        
            flag, rvecs, tvecs = cv2.solvePnP(object_points, image_points, mtx_np,dst_np)
            rvecs = rvecs.flatten()
            tvecs = tvecs.flatten()
            print('rvecs',rvecs)
            print('tvecs',tvecs)
            # Store the translation (i.e. position) information
            transform_translation_x = tvecs[0]
            transform_translation_y = tvecs[1]
            transform_translation_z = tvecs[2]

            # Store the rotation information
            #rotation_matrix = np.eye(3)
            rotation_matrix = cv2.Rodrigues(np.array(rvecs))[0]
            r = R.from_matrix(rotation_matrix)
            quat = r.as_quat()   

            # Quaternion format     
            transform_rotation_x = quat[0] 
            transform_rotation_y = quat[1] 
            transform_rotation_z = quat[2] 
            transform_rotation_w = quat[3] 

            # Euler angle format in radians
            roll_x, pitch_y, yaw_z = self.euler_from_quaternion(transform_rotation_x, 
                                                        transform_rotation_y, 
                                                        transform_rotation_z, 
                                                        transform_rotation_w)

            # roll_x = math.degrees(roll_x)
            # pitch_y = math.degrees(pitch_y)
            # yaw_z = math.degrees(yaw_z)
            # print("transform_translation_x: {}".format(transform_translation_x))
            # print("transform_translation_y: {}".format(transform_translation_y))
            # print("transform_translation_z: {}".format(transform_translation_z))
            # print("roll_x: {}".format(roll_x))
            # print("pitch_y: {}".format(pitch_y))
            # print("yaw_z: {}".format(yaw_z))
            # print()
            (topLeft, topRight, bottomRight, bottomLeft) = corners[i].reshape((4, 2))
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            #print("id {} ".format(marker_id))
            #print("at image position ",[cX,cY])
            #print("rotation matrix",rotation_matrix[0:3, 0:3] )
            #print("translation matrix",[transform_translation_x ,transform_translation_y,transform_translation_z])
            #print(obj_points[i])

            # Draw the axes on the marker
            #cv2.aruco.drawAxis(detectingImage , mtx, dst, rvecs, tvecs, 0.05)
            detectingImage = cv2.drawFrameAxes(detectingImage, mtx, dst, rvecs, tvecs, 0.05) 

    # Display the resulting frame

    #cv2.imshow('frame',detectingImage )
    #cv2.waitKey(3)
    return detectingImage
                

  def image_callback(self, msg: Image) -> None:
    self.get_logger().info("image_callback")
    try:
      self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        self.get_logger().error("Error converting ROS Image to OpenCV format: {0}".format(e))
        return

    (rows,cols,channels) = self.cv_image.shape
    if cols > 160 and rows > 160 :
      cv2.circle(self.cv_image, (80,80), 50, (255,255,255))

    #cv2.imshow("Image window", self.cv_image)
    #cv2.waitKey(3)
    #detectingImage = self.detect_pose_return_tf()

    try:
      self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(detectingImage))
    except Exception as e:
      print(e)

  def euler_from_quaternion(self, x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
        
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
        
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
        
    return roll_x, pitch_y, yaw_z # in radians

  def camera_coordinate_to_pixel_position(self,focal_x,focal_y,pixel_position_principal_point_x, \
            pixel_position_principal_point_y, P_camra_coordinate):
    ox = round(pixel_position_principal_point_x)
    oy = round(pixel_position_principal_point_y)
    xc = P_camra_coordinate[0]
    yc = P_camra_coordinate[1]
    zc = P_camra_coordinate[2] 
    sx = 1
    sy = 1
    x_im = -(focal_x*xc)/(sx*zc) + ox
    y_im = -(focal_y*yc)/(sy*zc) + oy
    return x_im, y_im






def main(args):
    rclpy.init(args=args)
    img_convert_publisher = image_converter()
    rclpy.spin(img_convert_publisher)
    img_convert_publisher.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main(sys.argv)