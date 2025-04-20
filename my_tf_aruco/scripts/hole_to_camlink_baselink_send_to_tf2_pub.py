#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import geometry_msgs
import tf2_geometry_msgs 
from nav_msgs.msg import Odometry

import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import math # Math library
from numpy.linalg import inv



class HoleToCamlinkTF(Node):
    hole_marker_side_length = 0.068 #measured by hand using blender on .dae file

    def __init__(self, hole_frame="hole_frame"):
        super().__init__('hole_to_camlink_tf_node')
        
        self.is_marker_detected = False
        self.is_camera_info_set = False
        self._hole_frame = hole_frame 
        self.publish_hole_tf_to_camera = False # True would publish tf to camera, False would mean publish tf to base_link      
        
        # Create a new `TransformStamped` object.
        # A `TransformStamped` object is a ROS message that represents a transformation between two frames.
        self.transform_stamped = tf2_geometry_msgs.TransformStamped()

        # This line sets the `child_frame_id` attribute of the `TransformStamped` object.
        # The `child_frame_id` attribute specifies the frame that is being transformed to.
        # In this case, the robot's base frame is being transformed to the `world` frame.
        self.transform_stamped.child_frame_id = self._hole_frame

        # For the TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)



        self.Timer = self.create_timer(
            1.0, self.timer_callback
        )



        # This line creates a new `TransformBroadcaster` object.
        # A `TransformBroadcaster` object is a ROS node that publishes TF messages.
        #self.br = TransformBroadcaster(self)
        self.publisher_to_tf2_pub = self.create_publisher(tf2_geometry_msgs.TransformStamped, '/hole_point_wrt_camera', 10)
        self.subscription_image = self.create_subscription(
                Image,
                '/wrist_rgbd_depth_sensor/image_aruco_frame',  
                self.image_callback, 10)
        self.subscription_camera_info = self.create_subscription( CameraInfo, '/wrist_rgbd_depth_sensor/camera_info', self.camera_info_callback, 10)
        self.publisher = self.create_publisher(Image, '/wrist_rgbd_depth_sensor/image_hole_frame', 1)
        self.cv_bridge = CvBridge()
        self.get_logger().info("hole_to_camlink_tf_node ready!!")
        

    def timer_callback(self):
        if self.publish_hole_tf_to_camera:
            self.broadcast_new_tf_to_camera()
        else:
            self.broadcast_new_tf_to_baselink()

    def broadcast_new_tf_to_baselink(self):
        self.transform_stamped.header.frame_id = "base_link"
        try:
            now = rclpy.time.Time()
            dest_frame = "camera_solution_frame"  #"wrist_rgbd_camera_depth_optical_frame" #
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
            hole_wrt_camera_pose = geometry_msgs.msg.PoseStamped()
            hole_wrt_camera_pose.pose.position.x = self.transform_translation_x
            hole_wrt_camera_pose.pose.position.y = self.transform_translation_y
            hole_wrt_camera_pose.pose.position.z = self.transform_translation_z
            
            hole_wrt_camera_pose.pose.orientation.x = self.transform_rotation_x
            hole_wrt_camera_pose.pose.orientation.y = self.transform_rotation_y
            hole_wrt_camera_pose.pose.orientation.z = self.transform_rotation_z     
            hole_wrt_camera_pose.pose.orientation.w = self.transform_rotation_w          
                 
            transform_baselink_hole_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(hole_wrt_camera_pose,transform_baselink_camera)   
            
            self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

            # Set the translation of the TF message.
            # The translation of the TF message is set to the current position of the robot.
            self.transform_stamped.transform.translation.x = transform_baselink_hole_pose_stamped.pose.position.x
            self.transform_stamped.transform.translation.y = transform_baselink_hole_pose_stamped.pose.position.y
            self.transform_stamped.transform.translation.z = transform_baselink_hole_pose_stamped.pose.position.z

            # Set the rotation of the TF message.
            # The rotation of the TF message is set to the current orientation of the robot.
            self.transform_stamped.transform.rotation.x = transform_baselink_hole_pose_stamped.pose.orientation.x
            self.transform_stamped.transform.rotation.y = transform_baselink_hole_pose_stamped.pose.orientation.y
            self.transform_stamped.transform.rotation.z = transform_baselink_hole_pose_stamped.pose.orientation.z
            self.transform_stamped.transform.rotation.w = transform_baselink_hole_pose_stamped.pose.orientation.w

            # Send (broadcast) the TF message.
            #self.br.sendTransform(self.transform_stamped)
            self.publisher_to_tf2_pub.publish(self.transform_stamped)
            self.get_logger().info("publishing tf from base_link to hole_frame")
        else:
            hole_wrt_camera_pose = geometry_msgs.msg.PoseStamped()
            hole_wrt_camera_pose.pose.position.x = 0.0
            hole_wrt_camera_pose.pose.position.y = 0.0
            hole_wrt_camera_pose.pose.position.z = 0.0
            r = R.from_matrix([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])          
            quat = r.as_quat() 
            hole_wrt_camera_pose.pose.orientation.x = quat[0]
            hole_wrt_camera_pose.pose.orientation.y = quat[1]
            hole_wrt_camera_pose.pose.orientation.z = quat[2]   
            hole_wrt_camera_pose.pose.orientation.w = quat[3]      
                 
            transform_baselink_hole_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(hole_wrt_camera_pose,transform_baselink_camera)   
            
            self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

            # Set the translation of the TF message.
            # The translation of the TF message is set to the current position of the robot.
            self.transform_stamped.transform.translation.x = transform_baselink_hole_pose_stamped.pose.position.x
            self.transform_stamped.transform.translation.y = transform_baselink_hole_pose_stamped.pose.position.y
            self.transform_stamped.transform.translation.z = transform_baselink_hole_pose_stamped.pose.position.z

            # Set the rotation of the TF message.
            # The rotation of the TF message is set to the current orientation of the robot.
            self.transform_stamped.transform.rotation.x = transform_baselink_hole_pose_stamped.pose.orientation.x
            self.transform_stamped.transform.rotation.y = transform_baselink_hole_pose_stamped.pose.orientation.y
            self.transform_stamped.transform.rotation.z = transform_baselink_hole_pose_stamped.pose.orientation.z
            self.transform_stamped.transform.rotation.w = transform_baselink_hole_pose_stamped.pose.orientation.w

            # Send (broadcast) the TF message.
            #self.br.sendTransform(self.transform_stamped)
            self.publisher_to_tf2_pub.publish(self.transform_stamped)
            self.get_logger().info("publishing identity tf from base_link to hole_frame")
            
        # Euler angle format in radians
        try:
            roll_x, pitch_y, yaw_z = self.euler_from_quaternion(self.transform_rotation_x, 
                                                                self.transform_rotation_y, 
                                                                self.transform_rotation_z, 
                                                                self.transform_rotation_w)        
            self.get_logger().info("TF base_link->hole_frame xyz=({:.3f},{:.3f},{:.3f}), row,pitch,yaw=({:.3f},{:.3f},{:.3f})".format( \
                self.transform_translation_x, self.transform_translation_y, self.transform_translation_z,roll_x, pitch_y, yaw_z))
        except AttributeError:
            pass


    def broadcast_new_tf_to_camera(self):
        """
        This function broadcasts a new TF message to the TF network.
        """
        self.transform_stamped.header.frame_id = "wrist_rgbd_camera_depth_optical_frame"
        if(self.is_marker_detected):
            # print('broadcast_new_tf')
            # Get the current odometry data.
            #position = self.cam_bot_odom.pose.pose.position
            #orientation = self.cam_bot_odom.pose.pose.orientation

            # Set the timestamp of the TF message.
            # The timestamp of the TF message is set to the current time.
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
            #self.br.sendTransform(self.transform_stamped)
            self.publisher_to_tf2_pub.publish(self.transform_stamped)
            self.get_logger().info("publishing identity tf from camera to hole_frame")
        else:
            self.get_logger().info("no hole detected. no hole frame published.")
        
        # Euler angle format in radians
        try:
            roll_x, pitch_y, yaw_z = self.euler_from_quaternion(self.transform_rotation_x, 
                                                                self.transform_rotation_y, 
                                                                self.transform_rotation_z, 
                                                                self.transform_rotation_w)        
            self.get_logger().info("TF wrist_rgbd_camera_depth_optical_frame->hole_frame xyz=({:.3f},{:.3f},{:.3f}), row,pitch,yaw=({:.3f},{:.3f},{:.3f})".format( \
                self.transform_translation_x, self.transform_translation_y, self.transform_translation_z,roll_x, pitch_y, yaw_z))
        except AttributeError:
            pass
    
    def detect_pose_return_tf(self):

        object_points = np.array([(-self.hole_marker_side_length/2., self.hole_marker_side_length/2., 0),\
                                    (self.hole_marker_side_length/2., self.hole_marker_side_length/2., 0),\
                                    (self.hole_marker_side_length/2., -self.hole_marker_side_length/2., 0),\
                                    (-self.hole_marker_side_length/2., -self.hole_marker_side_length/2., 0)],\
                                dtype=np.float32).reshape(4,1,3)   
        if not self.is_camera_info_set:
            return None
        
        if self.cv_image is None:
                print("failed to capture videos")
                return
        detectingImage = self.cv_image.copy() 

        # Detect hole in the video frame
        # return (list) bounding_box_ids, (what data type?) corners

        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY) 
        gray = cv2.medianBlur(gray, 5)      
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=30)
        if circles is not None:
            is_circle_found = True
            circles = np.uint16(np.around(circles))
            #print('circles',circles)
            for i in circles[0, :]:
                center = (i[0], i[1])

                # circle center
                cv2.circle(detectingImage, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(detectingImage, center, radius, (255, 0, 255), 3)

                top_left = (i[0] - radius, i[1] - radius)
                bottom_right = (i[0] + radius, i[1] + radius )
                #print(center, top_left, bottom_right)
                cv2.rectangle(detectingImage,top_left,bottom_right,(0,255,0),2)
                corners = (np.array([[[i[0] - radius,i[1] - radius], [i[0] + radius, i[1] - radius], [i[0] + radius, i[1] + radius], [i[0] - radius, i[1] + radius]]], 
                    dtype=np.float32),) 
                bounding_box_ids =  [[0]] 

        else:
            is_circle_found = False

        

        # Check that at least one hole marker was detected
        if is_circle_found: 
            self.is_marker_detected = True
            self.had_detected_marker = True
            num_markers = len(bounding_box_ids)
            #print('corners',corners)

            for i, marker_id in enumerate(bounding_box_ids):

                # TODO #if(bounding_box_ids is the chosen hole):
                realign_corners = np.zeros((4,2), dtype =np.float32)
                realign_corners[0] = corners[i][:,0,:].flatten()
                realign_corners[1] = corners[i][:,1,:].flatten()
                realign_corners[2] = corners[i][:,2,:].flatten()
                realign_corners[3] = corners[i][:,3,:].flatten()
                # print(realign_corners)  
                image_points = realign_corners.reshape(4,1,2)
                ## print(image_points)
            
                #flag, rvecs, tvecs = cv2.solvePnP(object_points, image_points, self.projection_matrix_k,dst)
                flag, rvecs, tvecs = cv2.solvePnP(object_points, image_points, self.projection_matrix_k,self.distortion_params)
                rvecs = rvecs.flatten()
                tvecs = tvecs.flatten()
                # print('rvecs',rvecs)
                # print('tvecs',tvecs)
                # Store the translation (i.e. position) information
                self.transform_translation_x = tvecs[0]
                self.transform_translation_y = tvecs[1]
                self.transform_translation_z = tvecs[2] 

                # Store the rotation information
                #rotation_matrix = np.eye(3)
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
                self.get_logger().info("marker id {} detected at xyz=({:.3f},{:.3f},{:.3f}), row,pitch,yaw=({:.3f},{:.3f},{:.3f}) w.r.t {}".format(marker_id,
                 self.transform_translation_x, self.transform_translation_y, self.transform_translation_z,roll_x, pitch_y, yaw_z,
                 self.transform_stamped.header.frame_id))


                (topLeft, topRight, bottomRight, bottomLeft) = corners[i].reshape((4, 2))
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Draw the axes on the marker
                detectingImage = cv2.drawFrameAxes(detectingImage, self.projection_matrix_k, self.distortion_params, rvecs, tvecs, 0.05)

                
        else:
            self.is_marker_detected = False

        # Display the resulting frame

        #cv2.imshow('frame',detectingImage )
        #cv2.waitKey(3)
        return detectingImage
                    

    def image_callback(self, msg: Image) -> None:
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Error converting ROS Image to OpenCV format: {0}".format(e))
            return

        (rows,cols,channels) = self.cv_image.shape


        detectingImage = self.detect_pose_return_tf()
        #cv2.imshow("Image window",  detectingImage)
        #cv2.waitKey(3)    
        if detectingImage is not None:
            try:
                image_message = self.cv_bridge.cv2_to_imgmsg(detectingImage, encoding="bgr8")
                self.publisher.publish(image_message)
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
    hole_to_cam_tf_obj = HoleToCamlinkTF()
    rclpy.spin(hole_to_cam_tf_obj)

    #TODO
    # - add subscription to /camera_info topic and get all camera parameters, instead of mock up camera param currently use
    # - fix the value to be correct value, and expect correct hole_frame's TF

if __name__ == '__main__':
    main()