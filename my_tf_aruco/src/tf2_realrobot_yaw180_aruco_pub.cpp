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

using namespace std::chrono_literals;

using std::placeholders::_1;

class Tf2Pub : public rclcpp::Node {
public:
  Tf2Pub() : Node("tf2_yaw180_pub_node") {

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscription_2_aruco_geometry =
        this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "aruco_point_wrt_camera", 10,
            std::bind(&Tf2Pub::aruco_geometry_callback, this, _1));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      subscription_2_aruco_geometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void aruco_geometry_callback(
      const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Call back");
    rclcpp::Time now = this->get_clock()->now();

    // This q, and p together form a Pose. This is aruco w.r.t camera Pose.
    tf2::Quaternion q_aruco_camera(
        msg->transform.rotation.x, msg->transform.rotation.y,
        msg->transform.rotation.z, msg->transform.rotation.w);
    tf2::Vector3 p_aruco_camera(msg->transform.translation.x,
                                msg->transform.translation.y,
                                msg->transform.translation.z);

    tf2::Transform transform_aruco_camera(q_aruco_camera, p_aruco_camera);
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, -3.14);
    tf2::Vector3 myVector(0, 0, 0);
    tf2::Transform myTransform(myQuaternion, myVector);
    tf2::Quaternion q_yaw180_aruco_camera = myTransform * q_aruco_camera;

    //------------ broadcast TF

    std::string fromFrameRel = msg->header.frame_id; // from parent to child
    std::string toFrameRel = msg->child_frame_id;    // child

    geometry_msgs::msg::TransformStamped msg_aruco_camera;
    rclcpp::Time now2 = this->get_clock()->now();
    msg_aruco_camera.header.stamp = now2;
    msg_aruco_camera.header.frame_id = fromFrameRel;
    msg_aruco_camera.child_frame_id = toFrameRel;
    msg_aruco_camera.transform.translation.x = p_aruco_camera.getX();
    msg_aruco_camera.transform.translation.y = p_aruco_camera.getY();
    msg_aruco_camera.transform.translation.z = p_aruco_camera.getZ();
    msg_aruco_camera.transform.rotation.x = q_yaw180_aruco_camera.getX();
    msg_aruco_camera.transform.rotation.y = q_yaw180_aruco_camera.getY();
    msg_aruco_camera.transform.rotation.z = q_yaw180_aruco_camera.getZ();
    msg_aruco_camera.transform.rotation.w = q_yaw180_aruco_camera.getW();
    tf_broadcaster_->sendTransform(msg_aruco_camera);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<Tf2Pub> tf2_pub_node = std::make_shared<Tf2Pub>();
  rclcpp::spin(tf2_pub_node);

  rclcpp::shutdown();
  return 0;
}
