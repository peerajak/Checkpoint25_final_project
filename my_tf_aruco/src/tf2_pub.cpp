#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <unistd.h>
#include <cmath>

#define FROM_BASE_TO_FAKE_CAMERA  // ifdef = base_link to fake_camera_frame, ifndef = aruco_frame to fake_camera_frame 

/*
run together with 
ros2 run my_tf_aruco aruco_to_camlink_send_to_tf2_pub.py
and
ros2 run my_tf_aruco aruco_to_camlink_tf_pub.py
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
      subscription_2_aruco_geometry = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "aruco_point_wrt_camera", 10, std::bind(&Tf2Pub::aruco_geometry_callback, this, _1));

    }

  private:
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_2_aruco_geometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void aruco_geometry_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
     RCLCPP_INFO(this->get_logger(), "Call back");
    rclcpp::Time now = this->get_clock()->now();
#ifdef FROM_BASE_TO_FAKE_CAMERA
    std::string fromFrame = "base_link";
    std::string toFrame = "aruco_frame";

    geometry_msgs::msg::TransformStamped tf_base_to_aruco_link ;
    try {
      tf_base_to_aruco_link = tf_buffer_->lookupTransform(toFrame,fromFrame,
                                                    tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), ex.what());
      return;
    }


    tf2::Quaternion q_base_to_aruco_link(tf_base_to_aruco_link.transform.rotation.x,
      tf_base_to_aruco_link.transform.rotation.y,
      tf_base_to_aruco_link.transform.rotation.z,
      tf_base_to_aruco_link.transform.rotation.w);
    tf2::Vector3 p_base_to_aruco_link(tf_base_to_aruco_link.transform.translation.x,
      tf_base_to_aruco_link.transform.translation.y,
      tf_base_to_aruco_link.transform.translation.z);
    tf2::Transform transform_base_to_aruco(q_base_to_aruco_link, p_base_to_aruco_link);
#endif
    // This q, and p together form a Pose. This is aruco w.r.t camera Pose.
    tf2::Quaternion q_aruco_camera(msg->transform.rotation.x,
                      msg->transform.rotation.y,
                      msg->transform.rotation.z,
                      msg->transform.rotation.w);
    tf2::Vector3 p_aruco_camera(msg->transform.translation.x,
                    msg->transform.translation.y,
                    msg->transform.translation.z);
    
    tf2::Transform transform_aruco_camera(q_aruco_camera, p_aruco_camera);
    tf2::Vector3 p_camera_aruco = transform_aruco_camera.inverse().getOrigin();
    tf2::Quaternion q_camera_aruco = transform_aruco_camera.inverse().getRotation();

#ifdef FROM_BASE_TO_FAKE_CAMERA
    tf2::Vector3 p_base_camera = transform_base_to_aruco.inverse() * p_camera_aruco;
    tf2::Quaternion q_base_camera = transform_base_to_aruco.inverse() * q_camera_aruco;
#endif

    //------------ broadcast TF 

#ifdef FROM_BASE_TO_FAKE_CAMERA
    std::string fromFrameRel = "base_link"; 
    std::string toFrameRel = "fake_camera_frame_from_base"; // TODO change Camera link name
    geometry_msgs::msg::TransformStamped msg_base_camera;
    rclcpp::Time now2 = this->get_clock()->now();
    msg_base_camera.header.stamp = now2;
    msg_base_camera.header.frame_id = fromFrameRel;
    msg_base_camera.child_frame_id = toFrameRel;
    msg_base_camera.transform.translation.x = p_base_camera.getX();
    msg_base_camera.transform.translation.y = p_base_camera.getY();
    msg_base_camera.transform.translation.z = p_base_camera.getZ();
    msg_base_camera.transform.rotation.x = q_base_camera.getX();
    msg_base_camera.transform.rotation.y = q_base_camera.getY();
    msg_base_camera.transform.rotation.z = q_base_camera.getZ();
    msg_base_camera.transform.rotation.w = q_base_camera.getW();
    tf_broadcaster_->sendTransform(msg_base_camera);
#else
    std::string fromFrameRel = "aruco_frame"; // TODO real robot is base_link
    std::string toFrameRel = "fake_camera_frame_from_aruco_frame"; // TODO change Camera link name
    geometry_msgs::msg::TransformStamped msg_aruco_camera;
    rclcpp::Time now2 = this->get_clock()->now();
    msg_aruco_camera.header.stamp = now2;
    msg_aruco_camera.header.frame_id = fromFrameRel;
    msg_aruco_camera.child_frame_id = toFrameRel;
    msg_aruco_camera.transform.translation.x = p_camera_aruco.getX();
    msg_aruco_camera.transform.translation.y = p_camera_aruco.getY();
    msg_aruco_camera.transform.translation.z = p_camera_aruco.getZ();
    msg_aruco_camera.transform.rotation.x = q_camera_aruco.getX();
    msg_aruco_camera.transform.rotation.y = q_camera_aruco.getY();
    msg_aruco_camera.transform.rotation.z = q_camera_aruco.getZ();
    msg_aruco_camera.transform.rotation.w = q_camera_aruco.getW();
    tf_broadcaster_->sendTransform(msg_aruco_camera);
#endif





  }
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<Tf2Pub> tf2_pub_node = std::make_shared<Tf2Pub>();
    rclcpp::spin(tf2_pub_node);

    rclcpp::shutdown();
    return 0;
}
