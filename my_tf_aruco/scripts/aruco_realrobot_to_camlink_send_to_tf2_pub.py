#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
# from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import geometry_msgs
import tf2_geometry_msgs 
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import math # Math library
from numpy.linalg import inv

# run together with 
# ros2 run my_tf_aruco tf2_pub_node
# and
# ros2 run my_tf_aruco aruco_to_camlink_tf_pub.py
# to create fake_camera_frame from aruco_frame

class ArucoToCamlinkTF(Node):
    # Dictionary that was used to generate the ArUco marker
    aruco_dictionary_name = "DICT_4X4_50"
    aruco_marker_side_length = 0.045 #correct value is 0.045 

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

    def __init__(self, aruco_frame="aruco_frame"):
        super().__init__('aruco_to_camlink_tf_node')
        self.is_marker_detected = False
        self.is_camera_info_set = False
        self._aruco_frame = aruco_frame        




        # For the TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.group_timer = MutuallyExclusiveCallbackGroup()
        self.Timer = self.create_timer(
            0.05, self.timer_callback, callback_group=self.group_timer
        )

        self.group_image = MutuallyExclusiveCallbackGroup()
        self.subscription_image = self.create_subscription(
                CompressedImage,
                '/D415/color/image_raw/compressed', 
                self.image_callback, qos_profile=1,callback_group=self.group_image)

        self.group_camera_info = MutuallyExclusiveCallbackGroup()
        self.subscription_camera_info = self.create_subscription( CameraInfo, '/D415/color/camera_info', self.camera_info_callback,\
             qos_profile=1, callback_group=self.group_camera_info)

        self.publisher_compressed = self.create_publisher(CompressedImage, '/D415/color/image_aruco/compressed', \
            qos_profile=1, )
        self.publisher_to_tf2_pub = self.create_publisher(tf2_geometry_msgs.TransformStamped, '/aruco_point_wrt_camera', 1)
        self.cv_bridge = CvBridge()
        self.get_logger().info("aruco_realrobot_to_camlink_send_to_tf2_pub ready!!")
        

    def timer_callback(self):
        self.get_logger().info("timer_callback")
        #self.broadcast_new_tf_to_camera()
        self.broadcast_new_tf_to_baselink()


    def broadcast_new_tf_to_baselink(self):
        self.transform_stamped = tf2_geometry_msgs.TransformStamped()
        try:
            now = rclpy.time.Time()
            dest_frame = "D415_color_optical_frame"
            origin_frame = "base_link"
            transform_baselink_camera = self.tf_buffer.lookup_transform(
                origin_frame,
                dest_frame,
                now)
        except TransformException as ex:
            self.get_logger().error(
                f'Could not transform {origin_frame} to {dest_frame}: {ex}')
            return None        
        if(self.is_marker_detected):
            try:
                aruco_wrt_camera_pose = geometry_msgs.msg.PoseStamped()
                aruco_wrt_camera_pose.pose.position.x = self.transform_translation_x
                aruco_wrt_camera_pose.pose.position.y = self.transform_translation_y
                aruco_wrt_camera_pose.pose.position.z = self.transform_translation_z
                
                aruco_wrt_camera_pose.pose.orientation.x = self.transform_rotation_x
                aruco_wrt_camera_pose.pose.orientation.y = self.transform_rotation_y
                aruco_wrt_camera_pose.pose.orientation.z = self.transform_rotation_z     
                aruco_wrt_camera_pose.pose.orientation.w = self.transform_rotation_w          
                    
                transform_baselink_aruco_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(aruco_wrt_camera_pose,transform_baselink_camera)   
                self.transform_stamped.header.frame_id = "base_link" #if marker detected, header frame is D415
                self.transform_stamped.child_frame_id = self._aruco_frame
                self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

                # Set the translation of the TF message.
                # The translation of the TF message is set to the current position of the robot.
                self.transform_stamped.transform.translation.x = transform_baselink_aruco_pose_stamped.pose.position.x
                self.transform_stamped.transform.translation.y = transform_baselink_aruco_pose_stamped.pose.position.y
                self.transform_stamped.transform.translation.z = transform_baselink_aruco_pose_stamped.pose.position.z

                # Set the rotation of the TF message.
                # The rotation of the TF message is set to the current orientation of the robot.
                self.transform_stamped.transform.rotation.x = transform_baselink_aruco_pose_stamped.pose.orientation.x
                self.transform_stamped.transform.rotation.y = transform_baselink_aruco_pose_stamped.pose.orientation.y
                self.transform_stamped.transform.rotation.z = transform_baselink_aruco_pose_stamped.pose.orientation.z
                self.transform_stamped.transform.rotation.w = transform_baselink_aruco_pose_stamped.pose.orientation.w

                # Send (broadcast) the TF message.
                self.publisher_to_tf2_pub.publish(self.transform_stamped)
                self.get_logger().info("publishing tf from base_link to aruco_frame")
            except AttributeError:
                pass
        else:
            try:
                aruco_wrt_camera_pose = geometry_msgs.msg.PoseStamped()
                aruco_wrt_camera_pose.pose.position.x = 0.0
                aruco_wrt_camera_pose.pose.position.y = 0.0
                aruco_wrt_camera_pose.pose.position.z = 0.0
                r = R.from_matrix([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])          
                quat = r.as_quat() 
                aruco_wrt_camera_pose.pose.orientation.x = quat[0]
                aruco_wrt_camera_pose.pose.orientation.y = quat[1]
                aruco_wrt_camera_pose.pose.orientation.z = quat[2]   
                aruco_wrt_camera_pose.pose.orientation.w = quat[3]      
                    
                transform_baselink_aruco_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(aruco_wrt_camera_pose,transform_baselink_camera)   
                self.transform_stamped.header.frame_id = "base_link" #if marker not detected, header frame is base_link
                self.transform_stamped.child_frame_id = self._aruco_frame
                self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

                # Set the translation of the TF message.
                # The translation of the TF message is set to the current position of the robot.
                self.transform_stamped.transform.translation.x = aruco_wrt_camera_pose.pose.position.x 
                self.transform_stamped.transform.translation.y = aruco_wrt_camera_pose.pose.position.y
                self.transform_stamped.transform.translation.z = aruco_wrt_camera_pose.pose.position.z 

                # Set the rotation of the TF message.
                # The rotation of the TF message is set to the current orientation of the robot.
                self.transform_stamped.transform.rotation.x = aruco_wrt_camera_pose.pose.orientation.x 
                self.transform_stamped.transform.rotation.y = aruco_wrt_camera_pose.pose.orientation.y
                self.transform_stamped.transform.rotation.z = aruco_wrt_camera_pose.pose.orientation.z
                self.transform_stamped.transform.rotation.w = aruco_wrt_camera_pose.pose.orientation.w

                # Send (broadcast) the TF message.
                self.publisher_to_tf2_pub.publish(self.transform_stamped)
                self.get_logger().info("publishing identity tf from base_link to aruco_frame")
            except AttributeError:
                pass
            
        # Euler angle format in radians
        try:
            roll_x, pitch_y, yaw_z = self.euler_from_quaternion(self.transform_rotation_x, 
                                                                self.transform_rotation_y, 
                                                                self.transform_rotation_z, 
                                                                self.transform_rotation_w)        
            self.get_logger().info("TF base_link->aruco_frame xyz=({:.3f},{:.3f},{:.3f}), row,pitch,yaw=({:.3f},{:.3f},{:.3f})".format( \
                self.transform_translation_x, self.transform_translation_y, self.transform_translation_z,roll_x, pitch_y, yaw_z))
        except AttributeError:
            pass      

    def broadcast_new_tf_to_camera(self):
        """
        This function broadcasts a new TF message to the TF network.
        """
        self.transform_stamped = tf2_geometry_msgs.TransformStamped()
        if(self.is_marker_detected):
            # print('broadcast_new_tf')
            # Get the current odometry data.
            #position = self.cam_bot_odom.pose.pose.position
            #orientation = self.cam_bot_odom.pose.pose.orientation

            # Set the timestamp of the TF message.
            # The timestamp of the TF message is set to the current time.   
            try:         
                self.transform_stamped.header.frame_id = "D415_color_optical_frame" #if marker detected, header frame is D415
                self.transform_stamped.child_frame_id = self._aruco_frame
                self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
            # Set the translation of the TF message.
            # The translation of the TF message is set to the current position of the robot.
                self.transform_stamped.transform.translation.x = self.transform_translation_x
                self.transform_stamped.transform.translation.y = self.transform_translation_y
                self.transform_stamped.transform.translation.z = self.transform_translation_z

                # Set the rotation of the TF message.
                # The rotation of the TF message is set to the current orientation of the robot.
                self.transform_stamped.transform.rotation.x = self.transform_rotation_x
                self.transform_stamped.transform.rotation.y = self.transform_rotation_y
                self.transform_stamped.transform.rotation.z = self.transform_rotation_z
                self.transform_stamped.transform.rotation.w = self.transform_rotation_w
                # Send (broadcast) the TF message.
                self.publisher_to_tf2_pub.publish(self.transform_stamped)
                self.get_logger().info("publishing tf from camera to aruco_frame")
            except AttributeError:
                pass


        else:            


            # Set the translation of the TF message.
            # The translation of the TF message is set to the current position of the robot.
            try:
                self.transform_stamped.header.frame_id = "base_link" #if marker not detected, header frame is base_link
                self.transform_stamped.child_frame_id = self._aruco_frame
                self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
                self.transform_stamped.transform.translation.x = 0.0
                self.transform_stamped.transform.translation.y = 0.0
                self.transform_stamped.transform.translation.z = 0.0      
                r = R.from_matrix([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])          
                quat = r.as_quat()   

                # Quaternion format     
                self.transform_rotation_x = quat[0] 
                self.transform_rotation_y = quat[1] 
                self.transform_rotation_z = quat[2] 
                self.transform_rotation_w = quat[3] 

                self.transform_stamped.transform.rotation.x = self.transform_rotation_x
                self.transform_stamped.transform.rotation.y = self.transform_rotation_y
                self.transform_stamped.transform.rotation.z = self.transform_rotation_z
                self.transform_stamped.transform.rotation.w = self.transform_rotation_w
                self.publisher_to_tf2_pub.publish(self.transform_stamped)
                self.get_logger().info("publishing tf from camera to aruco_frame")
            except AttributeError:
                pass
        
        # Euler angle format in radians
        try:
            roll_x, pitch_y, yaw_z = self.euler_from_quaternion(self.transform_rotation_x, 
                                                                self.transform_rotation_y, 
                                                                self.transform_rotation_z, 
                                                                self.transform_rotation_w)        
            self.get_logger().info("TF D415_color_optical_frame->aruco_frame xyz=({:.3f},{:.3f},{:.3f}), row,pitch,yaw=({:.3f},{:.3f},{:.3f})".format( \
                self.transform_translation_x, self.transform_translation_y, self.transform_translation_z,roll_x, pitch_y, yaw_z))
        except AttributeError:
            pass
    
    def detect_pose_return_tf(self):
        # Check that we have a valid ArUco marker
        if self.ARUCO_DICT.get(self.aruco_dictionary_name, None) is None:
            print("[INFO] ArUCo tag of is not supported")
            return None
        
        if not self.is_camera_info_set:
            return None
        
        mtx_real= np.array([ [306.80584716796875, 0.000000,214.4418487548828],[0.000000, 306.80584716796875, 124.9103012084961], [0., 0., 1.]], np.float32)
        mtx_test= np.array([ [306.80584716796875*0.9, 0.000000,214.4418487548828*1.05],[0.000000, 306.80584716796875*0.9, 124.9103012084961*1.25], [0., 0., 1.]], np.float32)
        mtx = mtx_test
        dst_np = np.array([0.062948, -0.273568, 0.005933, -0.001056, 0.000000], np.float32)   
        #dst_np = np.array([ 0.189572, -0.795616, 0.001088, -0.006897, 0.000000], np.float32)  
        prj_np = np.array([[761.265137, 0.000000, 311.720175, 0.000000],\
                           [0.000000, 764.304443, 215.883204, 0.000000],\
                           [0.000000, 0.000000, 1.000000, 0.000000]], np.float32)   
        #dst = dst_np 
        dst = np.zeros((5,), np.float32)  

        # Load the ArUco dictionary
        this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.aruco_dictionary_name])
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        
        # Start the video stream
        # TODO change cap to the topic image (self.cv_image)

        object_points = np.array([  (self.aruco_marker_side_length/2., -self.aruco_marker_side_length/2., 0),\
                                    (-self.aruco_marker_side_length/2., -self.aruco_marker_side_length/2., 0),\
                                    (-self.aruco_marker_side_length/2., self.aruco_marker_side_length/2., 0),\
                                    (self.aruco_marker_side_length/2., self.aruco_marker_side_length/2., 0)],\
                                dtype=np.float32).reshape(4,1,3)
        if self.cv_image is None:
                print("failed to capture videos")
                return

        detectingImage = self.cv_image.copy() 

        # Image brightness adjustment for better detection
        detectingImage_np = np.zeros(detectingImage.shape, detectingImage.dtype)
        alpha = 4.2 #great value at 4.2#float  Simple contrast control 
        beta = 60  #great value at 60  #integer Simple brightness control
        for y in range(detectingImage.shape[0]):
            for x in range(detectingImage.shape[1]):
                for c in range(detectingImage.shape[2]):
                    detectingImage_np[y,x,c] = np.clip(alpha*detectingImage[y,x,c] + beta, 0, 255)

        # Detect ArUco markers in the video frame

        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            detectingImage_np, this_aruco_dictionary, parameters=this_aruco_parameters,
            cameraMatrix=mtx, distCoeff=dst)
        # (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
        #     detectingImage_np, this_aruco_dictionary, parameters=this_aruco_parameters,
        #     cameraMatrix=self.projection_matrix_k, distCoeff=self.distortion_params)

        # Check that at least one ArUco marker was detected
        if marker_ids is not None: 
            self.is_marker_detected = True
            self.had_detected_marker = True
            num_markers = len(marker_ids)
            detectingImage = cv2.aruco.drawDetectedMarkers(detectingImage , corners)
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
                # print(realign_corners)  
                image_points = realign_corners.reshape(4,1,2)
                ## print(image_points)
            
                #flag, rvecs, tvecs = cv2.solvePnP(object_points, image_points, mtx,dst)
                flag, rvecs, tvecs = cv2.solvePnP(object_points, image_points, mtx, dst, useExtrinsicGuess=False,flags=cv2.SOLVEPNP_ITERATIVE)
                #flag, rvecs, tvecs = cv2.solvePnP(object_points, image_points, self.projection_matrix_k,self.distortion_params)
                rvecs = rvecs.flatten()
                tvecs = tvecs.flatten()
                # print('tvecs',tvecs)

                # Store the translation (i.e. position) information
                self.transform_translation_x = tvecs[0]
                self.transform_translation_y = tvecs[1]
                self.transform_translation_z = tvecs[2]

                # Store the rotation information
                rotation_matrix = cv2.Rodrigues(np.array(rvecs))[0]
                r = R.from_matrix(rotation_matrix)
                quat = r.as_quat()   

                # Quaternion format     
                self.transform_rotation_x = quat[0] 
                self.transform_rotation_y = quat[1] 
                self.transform_rotation_z = quat[2] 
                self.transform_rotation_w = quat[3] 

                # Euler angle format in radians
                roll_x, pitch_y, yaw_z = self.euler_from_quaternion(self.transform_rotation_x, 
                                                            self.transform_rotation_y, 
                                                            self.transform_rotation_z, 
                                                            self.transform_rotation_w)
                try:
                    self.get_logger().info("marker id {} detected at xyz=({:.3f},{:.3f},{:.3f}), row,pitch,yaw=({:.3f},{:.3f},{:.3f}) w.r.t {}".format(marker_id,
                 self.transform_translation_x, self.transform_translation_y, self.transform_translation_z,roll_x, pitch_y, yaw_z,
                 self.transform_stamped.header.frame_id))
                except AttributeError:
                    pass


                (topLeft, topRight, bottomRight, bottomLeft) = corners[i].reshape((4, 2))
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)


                # Draw the axes on the marker
                detectingImage =  cv2.aruco.drawAxis(detectingImage , mtx,dst, rvecs, tvecs, 0.05)
                #detectingImage = cv2.drawFrameAxes(detectingImage, self.projection_matrix_k, self.distortion_params, rvecs, tvecs, 0.05)
                
        else:
            self.is_marker_detected = False

        # Display the resulting frame

        #cv2.imshow('frame',detectingImage )
        #cv2.waitKey(3)
        return detectingImage
                    

    def image_callback(self, msg: CompressedImage) -> None:
        self.get_logger().info("image_callback")
        try:
            self.cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error("Error converting CompressedImage Image to OpenCV format: {0}".format(e))
            return

        (rows,cols,channels) = self.cv_image.shape


        #cv2.imshow("Image window", self.cv_image)
        #cv2.waitKey(3)
        detectingImage = self.detect_pose_return_tf()
        
        if detectingImage is not None:
            try:
                image_message_compressed = self.cv_bridge.cv2_to_compressed_imgmsg(detectingImage)
                self.publisher_compressed.publish(image_message_compressed)

            except Exception as e:
                print(e)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.is_camera_info_set = True
        self.distortion_params = np.zeros((5,), np.float32) 
        self.distortion_params[0] = msg.d[0] 
        self.distortion_params[1] = msg.d[1] 
        self.distortion_params[2] = msg.d[2] 
        self.distortion_params[3] = msg.d[3] 
        self.distortion_params[4] = msg.d[4] 

        self.projection_matrix_k = np.zeros((3,3), np.float32)
        self.projection_matrix_k[0,0] = msg.k[0]
        self.projection_matrix_k[0,1] = msg.k[1]
        self.projection_matrix_k[0,2] = msg.k[2]
        self.projection_matrix_k[1,0] = msg.k[3]
        self.projection_matrix_k[1,1] = msg.k[4]
        self.projection_matrix_k[1,2] = msg.k[5]
        self.projection_matrix_k[2,0] = msg.k[6]
        self.projection_matrix_k[2,1] = msg.k[7]
        self.projection_matrix_k[2,2] = msg.k[8]

        self.projection_matrix_p = np.zeros((3,4), np.float32)
        self.projection_matrix_p[0,0] = msg.p[0]
        self.projection_matrix_p[0,1] = msg.p[1]
        self.projection_matrix_p[0,2] = msg.p[2]
        self.projection_matrix_p[0,3] = msg.p[3]
        self.projection_matrix_p[1,0] = msg.p[4]
        self.projection_matrix_p[1,1] = msg.p[5]
        self.projection_matrix_p[1,2] = msg.p[6]
        self.projection_matrix_p[1,3] = msg.p[7]
        self.projection_matrix_p[2,0] = msg.p[8]
        self.projection_matrix_p[2,1] = msg.p[9]
        self.projection_matrix_p[2,2] = msg.p[10]
        self.projection_matrix_p[2,3] = msg.p[11]

        #print(self.distortion_params)
        #print(self.projection_matrix_k)
        #print(self.projection_matrix_p)
        


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




def main(args=None):

    rclpy.init()

    aruco_to_cam_tf_obj = ArucoToCamlinkTF()
    executor = MultiThreadedExecutor()
    executor.add_node(aruco_to_cam_tf_obj)
    executor.spin()


if __name__ == '__main__':
    main()