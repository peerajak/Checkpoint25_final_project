#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

/*
run together with
ros2 run my_tf_aruco aruco_to_camlink_send_to_tf2_pub.py
and
ros2 run my_tf_aruco aruco_to_camlink_tf_pub.py
to create fake_camera_frame from aruco_frame
*/
//#define ARUCO_TO_CAM

using namespace std::chrono_literals;

using std::placeholders::_1;

class Tf2Pub : public rclcpp::Node {
public:
  Tf2Pub() : Node("tf2_pub_node") {

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscription_2_aruco_geometry =
        this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "aruco_point_wrt_camera", 10,
            std::bind(&Tf2Pub::aruco_geometry_callback, this, _1));
    // timer_ = this->create_wall_timer(500ms,
    //                                  std::bind(&Tf2Pub::timer_callback,
    //                                  this));
    // #ifdef ARUCO_TO_CAM
    //     //is_aruco_to_cam_callback_ = false;
    // #endif
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      subscription_2_aruco_geometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // rclcpp::TimerBase::SharedPtr timer_;
  //  #ifdef ARUCO_TO_CAM
  //  //   geometry_msgs::msg::TransformStamped msg_aruco_camera_;
  //  //   bool is_aruco_to_cam_callback_;
  //  #endif

  //   void timer_callback() {
  // #ifdef ARUCO_TO_CAM
  //     if (is_aruco_to_cam_callback_)
  //       tf_broadcaster_->sendTransform(msg_aruco_camera_);
  //      RCLCPP_INFO(this->get_logger(), "msg_aruco_camera_ sent");
  // #endif
  //   }

  void aruco_geometry_callback(
      const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Call back");
    rclcpp::Time now = this->get_clock()->now();

    std::string fromFrame = "base_link";
    std::string toFrame = "aruco_link";

    geometry_msgs::msg::TransformStamped tf_base_to_aruco_link;
    try {
      tf_base_to_aruco_link =
          tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), ex.what());
      return;
    }

    tf2::Quaternion q_base_to_aruco_link(
        tf_base_to_aruco_link.transform.rotation.x,
        tf_base_to_aruco_link.transform.rotation.y,
        tf_base_to_aruco_link.transform.rotation.z,
        tf_base_to_aruco_link.transform.rotation.w);
    tf2::Vector3 p_base_to_aruco_link(
        tf_base_to_aruco_link.transform.translation.x,
        tf_base_to_aruco_link.transform.translation.y,
        tf_base_to_aruco_link.transform.translation.z);
    tf2::Transform transform_base_to_aruco(q_base_to_aruco_link,
                                           p_base_to_aruco_link);

    // This q, and p together form a Pose. This is aruco w.r.t camera Pose.
    tf2::Quaternion q_arucoFrame_camera( // arrow from child to parent
        msg->transform.rotation.x, msg->transform.rotation.y,
        msg->transform.rotation.z, msg->transform.rotation.w);
    tf2::Vector3 p_arucoFrame_camera(msg->transform.translation.x,
                                     msg->transform.translation.y,
                                     msg->transform.translation.z);

    tf2::Transform transform_arucoFrame_camera(q_arucoFrame_camera,
                                               p_arucoFrame_camera);
    tf2::Vector3 p_camera_aruco =
        transform_arucoFrame_camera.inverse().getOrigin();
    tf2::Quaternion q_camera_aruco =
        transform_arucoFrame_camera.inverse().getRotation();

    tf2::Vector3 p_base_camera =
        transform_base_to_aruco.inverse() * p_arucoFrame_camera;
    tf2::Quaternion q_base_camera =
        transform_base_to_aruco.inverse() * q_arucoFrame_camera;

    /*------------ broadcast TF
#ifdef ARUCO_TO_CAM

    std::string fromFrameRel0 = "aruco_link"; // from parent to child
    std::string toFrameRel0 = "D415_color_optical_frame"; // child
    geometry_msgs::msg::TransformStamped msg_arucoFrame_camera_;
    rclcpp::Time now0 = this->get_clock()->now();
    msg_arucoFrame_camera_.header.stamp = now0;
    msg_arucoFrame_camera_.header.frame_id = fromFrameRel0;translation.
    msg_arucoFrame_camera_.child_frame_id = toFrameRel0;
    msg_arucoFrame_camera_.transform.translation.x = p_arucoFrame_camera.getX();
    msg_arucoFrame_camera_.transform.translation.y = p_arucoFrame_camera.getY();
    msg_arucoFrame_camera_.transform.translation.z = p_arucoFrame_camera.getZ();
    msg_arucoFrame_camera_.transform.rotation.x = q_arucoFrame_camera.getX();
    msg_arucoFrame_camera_.transform.rotation.y = q_arucoFrame_camera.getY();
    msg_arucoFrame_camera_.transform.rotation.z = q_arucoFrame_camera.getZ();
    msg_arucoFrame_camera_.transform.rotation.w = q_arucoFrame_camera.getW();
    tf_broadcaster_->sendTransform(msg_arucoFrame_camera);
    // is_aruco_to_cam_callback_ = true;
#else */
    if(msg->header.frame_id == "D415_color_optical_frame") { // meaning aruco marker detected
        std::string fromFrameRel1 = "base_link"; // from parent to child
        std::string toFrameRel1 = "D415_color_optical_frame"; // child
        geometry_msgs::msg::TransformStamped msg_base_camera;
        rclcpp::Time now1 = this->get_clock()->now();
        msg_base_camera.header.stamp = now1;
        msg_base_camera.header.frame_id = fromFrameRel1;
        msg_base_camera.child_frame_id = toFrameRel1;
        msg_base_camera.transform.translation.x = p_base_camera.getX();
        msg_base_camera.transform.translation.y = p_base_camera.getY();
        msg_base_camera.transform.translation.z = p_base_camera.getZ();
        msg_base_camera.transform.rotation.x = q_base_camera.getX();
        msg_base_camera.transform.rotation.y = q_base_camera.getY();
        msg_base_camera.transform.rotation.z = q_base_camera.getZ();
        msg_base_camera.transform.rotation.w = q_base_camera.getW();
        tf_broadcaster_->sendTransform(msg_base_camera);
    } else {
        std::string fromFrameRel1 = "base_link"; // from parent to child
        std::string toFrameRel1 = "D415_color_optical_frame"; // child
        geometry_msgs::msg::TransformStamped msg_base_camera;
        rclcpp::Time now1 = this->get_clock()->now();
        msg_base_camera.header.stamp = now1;
        msg_base_camera.header.frame_id = fromFrameRel1;
        msg_base_camera.child_frame_id = toFrameRel1;
        msg_base_camera.transform.translation.x = msg->transform.translation.x;
        msg_base_camera.transform.translation.y = msg->transform.translation.y;
        msg_base_camera.transform.translation.z = msg->transform.translation.z;
        msg_base_camera.transform.rotation.x = msg->transform.rotation.x;
        msg_base_camera.transform.rotation.y = msg->transform.rotation.y;
        msg_base_camera.transform.rotation.z = msg->transform.rotation.z;
        msg_base_camera.transform.rotation.w = msg->transform.rotation.w;
        tf_broadcaster_->sendTransform(msg_base_camera);    
    }
//#endif
    std::string fromFrameRel2 =
        "D415_color_optical_frame";          // from parent to child
    std::string toFrameRel2 = "aruco_frame"; // child
    geometry_msgs::msg::TransformStamped msg_camera_aruco;
    rclcpp::Time now2 = this->get_clock()->now();
    msg_camera_aruco.header.stamp = now2;
    msg_camera_aruco.header.frame_id = fromFrameRel2;
    msg_camera_aruco.child_frame_id = toFrameRel2;
    msg_camera_aruco.transform.translation.x = p_camera_aruco.getX();
    msg_camera_aruco.transform.translation.y = p_camera_aruco.getY();
    msg_camera_aruco.transform.translation.z = p_camera_aruco.getZ();
    msg_camera_aruco.transform.rotation.x = q_camera_aruco.getX();
    msg_camera_aruco.transform.rotation.y = q_camera_aruco.getY();
    msg_camera_aruco.transform.rotation.z = q_camera_aruco.getZ();
    msg_camera_aruco.transform.rotation.w = q_camera_aruco.getW();
    tf_broadcaster_->sendTransform(msg_camera_aruco);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<Tf2Pub> tf2_pub_node = std::make_shared<Tf2Pub>();
  rclcpp::spin(tf2_pub_node);

  rclcpp::shutdown();
  return 0;
}
