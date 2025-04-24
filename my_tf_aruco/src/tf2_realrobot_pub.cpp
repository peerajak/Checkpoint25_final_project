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
to create fake_camera_frame from aruco_frame
*/

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
      std::bind(&Tf2Pub::aruco_geometry_callback_aruco_camera, this, _1));

   /* subscription_2_aruco_geometry =
        this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "aruco_point_wrt_camera", 10,
            std::bind(&Tf2Pub::aruco_geometry_callback_base_camera, this, _1)); */
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      subscription_2_aruco_geometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void aruco_geometry_callback_aruco_camera(
      const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "aruco_geometry_callback_aruco_camera");

    //------------ broadcast aruco_frame TF

    geometry_msgs::msg::TransformStamped msg_aruco_camera;
    rclcpp::Time now = this->get_clock()->now();
    msg_aruco_camera.header.stamp = now;
    msg_aruco_camera.header.frame_id = msg->header.frame_id;
    msg_aruco_camera.child_frame_id = msg->child_frame_id;
    msg_aruco_camera.transform.translation.x = msg->transform.translation.x;
    msg_aruco_camera.transform.translation.y = msg->transform.translation.y;
    msg_aruco_camera.transform.translation.z = msg->transform.translation.z;
    msg_aruco_camera.transform.rotation.x = msg->transform.rotation.x;
    msg_aruco_camera.transform.rotation.y = msg->transform.rotation.y;
    msg_aruco_camera.transform.rotation.z = msg->transform.rotation.z;
    msg_aruco_camera.transform.rotation.w = msg->transform.rotation.w;
    tf_broadcaster_->sendTransform(msg_aruco_camera);
  }

  void aruco_geometry_callback_base_camera(
      const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "aruco_geometry_callback_base_camera");

    // 1. Let rg2_gripper_aruco_link be a known avalue, find transformation from
    // base to rg2_gripper_aruco_link
    std::string fromFrame = "base_link";            // parent
    std::string toFrame = "rg2_gripper_aruco_link"; // child

    geometry_msgs::msg::TransformStamped tf_aruco_to_base_link;
    try {
      tf_aruco_to_base_link =
          tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), ex.what());
      return;
    } // checked passed

    // 2. find the inverse transformation of aruco_camera, ie. camera_aruco
    // (child_parent format)
    tf2::Quaternion q_aruco_camera( // arrow from child to parent
        msg->transform.rotation.x, msg->transform.rotation.y,
        msg->transform.rotation.z, msg->transform.rotation.w);
    tf2::Vector3 p_aruco_camera(msg->transform.translation.x,
                                msg->transform.translation.y,
                                msg->transform.translation.z);
    tf2::Transform transform_aruco_camera(q_aruco_camera, p_aruco_camera);
    tf2::Vector3 p_camera_aruco = transform_aruco_camera.inverse().getOrigin();
    tf2::Quaternion q_camera_aruco =
        transform_aruco_camera.inverse().getRotation();

    // tf2::Quaternion q_camera_aruco = q_aruco_camera.inverse();// this is also
    // ok

    // 3. prepare p,q base_camera from 1) and 2)
    tf2::Quaternion q_aruco_to_base_link(
        tf_aruco_to_base_link.transform.rotation.x,
        tf_aruco_to_base_link.transform.rotation.y,
        tf_aruco_to_base_link.transform.rotation.z,
        tf_aruco_to_base_link.transform.rotation.w);
    tf2::Vector3 p_aruco_to_base_link(
        tf_aruco_to_base_link.transform.translation.x,
        tf_aruco_to_base_link.transform.translation.y,
        tf_aruco_to_base_link.transform.translation.z);
    tf2::Transform transform_aruco_to_base(q_aruco_to_base_link,
                                           p_aruco_to_base_link);
    tf2::Vector3 p_base_camera = transform_aruco_to_base * p_camera_aruco;
    tf2::Quaternion q_base_camera = transform_aruco_to_base * q_camera_aruco;

    // 4. broadcast new TF camera_solution_frame from 3)
    std::string fromFrameRel1 =
        "base_link"; // rg2_gripper_aruco_link from parent to child
    std::string toFrameRel1 =
        "camera_solution_frame"; //"camera_solution_frame"; // child
    geometry_msgs::msg::TransformStamped msg_camera_base;
    rclcpp::Time now1 = this->get_clock()->now();
    msg_camera_base.header.stamp = now1;
    msg_camera_base.header.frame_id = fromFrameRel1.c_str();
    msg_camera_base.child_frame_id = toFrameRel1.c_str();
    msg_camera_base.transform.translation.x = p_base_camera.getX();
    msg_camera_base.transform.translation.y = p_base_camera.getY();
    msg_camera_base.transform.translation.z = p_base_camera.getZ();
    msg_camera_base.transform.rotation.x = q_base_camera.getX();
    msg_camera_base.transform.rotation.y = q_base_camera.getY();
    msg_camera_base.transform.rotation.z = q_base_camera.getZ();
    msg_camera_base.transform.rotation.w = q_base_camera.getW();
    tf_broadcaster_->sendTransform(msg_camera_base);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<Tf2Pub> tf2_pub_node = std::make_shared<Tf2Pub>();
  rclcpp::spin(tf2_pub_node);

  rclcpp::shutdown();
  return 0;
}