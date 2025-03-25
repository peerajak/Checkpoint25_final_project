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

//#define TOOL0_TO_ARUCO
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
    is_calibrated = false;
    // msg_aruco_tool0_tx = 0;
    // msg_aruco_tool0_ty = 0;
    // msg_aruco_tool0_tz = 0;
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscription_2_aruco_geometry =
        this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "aruco_point_wrt_camera", 10,
            std::bind(&Tf2Pub::aruco_geometry_callback, this, _1));
    // create a timer to keep publishing the tf
    //  callback_group_1_timer_broadcast = this->create_callback_group(
    //      rclcpp::CallbackGroupType::MutuallyExclusive);
    //  callback_group_2_timer_elapse = this->create_callback_group(
    //      rclcpp::CallbackGroupType::MutuallyExclusive);
    //  timer_broadcast_ = this->create_wall_timer(
    //      500ms, std::bind(&Tf2Pub::timer_broadcast_callback, this),
    //      callback_group_1_timer_broadcast);
    //  timer_elapse_ = this->create_wall_timer(
    //      5s, std::bind(&Tf2Pub::timer_elapse_callback, this),
    //      callback_group_2_timer_elapse);
  }

private:
  bool is_calibrated;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      subscription_2_aruco_geometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  //   rclcpp::CallbackGroup::SharedPtr
  //   callback_group_1_timer_broadcast,callback_group_2_timer_elapse;
  //   rclcpp::TimerBase::SharedPtr timer_broadcast_, timer_elapse_;
  //   double msg_aruco_tool0_tx,msg_aruco_tool0_ty,msg_aruco_tool0_tz;
  //   double
  //   prev_msg_aruco_tool0_tx,prev_msg_aruco_tool0_ty,prev_msg_aruco_tool0_tz;
  //   double
  //   msg_aruco_tool0_rx,msg_aruco_tool0_ry,msg_aruco_tool0_rz,msg_aruco_tool0_rw;
  //   double
  //   prev_msg_aruco_tool0_rx,prev_msg_aruco_tool0_ry,prev_msg_aruco_tool0_rz,prev_msg_aruco_tool0_rw;

  void aruco_geometry_callback(
      const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Call back");
    rclcpp::Time now = this->get_clock()->now();

    is_calibrated = false;

    // This q, and p together form a Pose. This is aruco w.r.t camera Pose.
    tf2::Quaternion q_aruco_camera(
        msg->transform.rotation.x, msg->transform.rotation.y,
        msg->transform.rotation.z, msg->transform.rotation.w);
    tf2::Vector3 p_aruco_camera(msg->transform.translation.x,
                                msg->transform.translation.y,
                                msg->transform.translation.z);

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
    msg_aruco_camera.transform.rotation.x = q_aruco_camera.getX();
    msg_aruco_camera.transform.rotation.y = q_aruco_camera.getY();
    msg_aruco_camera.transform.rotation.z = q_aruco_camera.getZ();
    msg_aruco_camera.transform.rotation.w = q_aruco_camera.getW();
    tf_broadcaster_->sendTransform(msg_aruco_camera);
#ifdef TOOL0_TO_ARUCO
    // 1. lookup the transform from tool0 to D415_color_optical_frame
    std::string fromFrame = msg->header.frame_id; // from parent to child
    std::string toFrame = "tool0";                // child

    geometry_msgs::msg::TransformStamped tf_camera_to_tool0;
    try {
      tf_camera_to_tool0 =
          tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrame.c_str(), fromFrame.c_str(), ex.what());
      return;
    }

    tf2::Quaternion q_camera_to_tool0(tf_camera_to_tool0.transform.rotation.x,
                                      tf_camera_to_tool0.transform.rotation.y,
                                      tf_camera_to_tool0.transform.rotation.z,
                                      tf_camera_to_tool0.transform.rotation.w);
    tf2::Vector3 p_camera_to_tool0(tf_camera_to_tool0.transform.translation.x,
                                   tf_camera_to_tool0.transform.translation.y,
                                   tf_camera_to_tool0.transform.translation.z);
    tf2::Transform transform_camera_to_tool0(q_camera_to_tool0,
                                             p_camera_to_tool0);

    // 2. compute the p_aruco_tool0, and q_aruco_tool0
    tf2::Vector3 p_aruco_tool0 = transform_camera_to_tool0 * p_aruco_camera;
    tf2::Quaternion q_aruco_tool0 = transform_camera_to_tool0 * q_aruco_camera;

    // msg_aruco_tool0_tx = p_aruco_tool0.getX();
    // msg_aruco_tool0_ty = p_aruco_tool0.getY();
    // msg_aruco_tool0_tz = p_aruco_tool0.getZ();
    // msg_aruco_tool0_rx = q_aruco_tool0.getX();
    // msg_aruco_tool0_ry = q_aruco_tool0.getY();
    // msg_aruco_tool0_rz = q_aruco_tool0.getZ();
    // msg_aruco_tool0_rw = q_aruco_tool0.getW();

    geometry_msgs::msg::TransformStamped msg_aruco_tool0;
    rclcpp::Time now3 = this->get_clock()->now();
    msg_aruco_tool0.header.stamp = now3;
    msg_aruco_tool0.header.frame_id = "tool0";
    msg_aruco_tool0.child_frame_id = "aruco_frame";
    msg_aruco_tool0.transform.translation.x = p_aruco_tool0.getX();
    msg_aruco_tool0.transform.translation.y = p_aruco_tool0.getY();
    msg_aruco_tool0.transform.translation.z = p_aruco_tool0.getZ();
    msg_aruco_tool0.transform.rotation.x = q_aruco_tool0.getX();
    msg_aruco_tool0.transform.rotation.y = q_aruco_tool0.getY();
    msg_aruco_tool0.transform.rotation.z = q_aruco_tool0.getZ();
    msg_aruco_tool0.transform.rotation.w = q_aruco_tool0.getW();
    tf_broadcaster_->sendTransform(msg_aruco_tool0);
#endif
  }

  //   void timer_elapse_callback () {// to do mature stable tf checking
  //     double thres = 0.001;
  //     if(std::abs(prev_msg_aruco_tool0_tx - msg_aruco_tool0_tx) < thres &&
  //     std::abs(prev_msg_aruco_tool0_ty - msg_aruco_tool0_ty) < thres &&
  //     std::abs(prev_msg_aruco_tool0_tz - msg_aruco_tool0_tz) < thres &&
  //     std::abs(prev_msg_aruco_tool0_rx - msg_aruco_tool0_rx) < thres &&
  //     std::abs(prev_msg_aruco_tool0_ry - msg_aruco_tool0_ry) < thres &&
  //     std::abs(prev_msg_aruco_tool0_rz - msg_aruco_tool0_rz) < thres &&
  //     std::abs(prev_msg_aruco_tool0_rw - msg_aruco_tool0_rw) < thres &&
  //     msg_aruco_tool0_tx != 0 && msg_aruco_tool0_ty !=0 && msg_aruco_tool0_tz
  //     !=0  ){
  //        is_calibrated = true;
  //        prev_msg_aruco_tool0_tx = msg_aruco_tool0_tx;
  //        prev_msg_aruco_tool0_ty = msg_aruco_tool0_ty;
  //        prev_msg_aruco_tool0_tz = msg_aruco_tool0_tz;
  //        prev_msg_aruco_tool0_rx = msg_aruco_tool0_rx;
  //        prev_msg_aruco_tool0_ry = msg_aruco_tool0_ry;
  //        prev_msg_aruco_tool0_rz = msg_aruco_tool0_rz;
  //        prev_msg_aruco_tool0_rw = msg_aruco_tool0_rw;
  //     }

  //   }

  //   void timer_broadcast_callback() {//once the tf is matured by
  //   timer_elaspe_callback
  //     if(is_calibrated){
  //         geometry_msgs::msg::TransformStamped msg_aruco_tool0;
  //         rclcpp::Time now3 = this->get_clock()->now();
  //         msg_aruco_tool0.header.stamp = now3;
  //         msg_aruco_tool0.header.frame_id = "tool0";
  //         msg_aruco_tool0.child_frame_id = "aruco_frame";
  //         msg_aruco_tool0.transform.translation.x = prev_msg_aruco_tool0_tx;
  //         msg_aruco_tool0.transform.translation.y = prev_msg_aruco_tool0_ty;
  //         msg_aruco_tool0.transform.translation.z = prev_msg_aruco_tool0_tz;
  //         msg_aruco_tool0.transform.rotation.x = prev_msg_aruco_tool0_rx;
  //         msg_aruco_tool0.transform.rotation.y = prev_msg_aruco_tool0_ry;
  //         msg_aruco_tool0.transform.rotation.z = prev_msg_aruco_tool0_rz;
  //         msg_aruco_tool0.transform.rotation.w = prev_msg_aruco_tool0_rw;
  //         tf_broadcaster_->sendTransform(msg_aruco_tool0);

  //     }
  //   }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<Tf2Pub> tf2_pub_node = std::make_shared<Tf2Pub>();
  rclcpp::spin(tf2_pub_node);

  rclcpp::shutdown();
  return 0;
}
