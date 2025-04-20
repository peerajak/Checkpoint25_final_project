#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>
#include <memory>
#include <unistd.h>

#define CALIBRATING true
#define PUBLISING_CALIBRATED false
#define SLEEPTIME 3
using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

/*  USAGE
 *   - To change to calibrating mode
 *   ros2 service call tf2_hole_pub_service std_srvs/srv/SetBool data:\ true
 *   - To change to publish calibrated camera TF mode
 *   ros2 service call tf2_hole_pub_service std_srvs/srv/SetBool data:\ false
 **/

static const rclcpp::Logger LOGGER = rclcpp::get_logger("tf2_hole_pub_service_node");

class Tf2PubServiceNode : public rclcpp::Node {
public:
  Tf2PubServiceNode(rclcpp::NodeOptions &node_options)
      : Node("tf2_hole_pub_service_node", node_options) {
    this->is_calibrating = CALIBRATING;
    this->was_calibrated_before = false;
    node_ = std::make_shared<rclcpp::Node>("tf2_hole_pub_service");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    srv_ = create_service<SetBool>(
        "tf2_hole_pub_service",
        std::bind(&Tf2PubServiceNode::service_state_callback, this, _1, _2));
    calibrated_msg_base_camera_ =
        std::make_shared<geometry_msgs::msg::TransformStamped>();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscription_2_hole_geometry =
        this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "hole_point_wrt_camera", 10,
            std::bind(&Tf2PubServiceNode::hole_geometry_service_callback, this,
                      _1));

    timer_ = this->create_wall_timer(
        500ms, std::bind(&Tf2PubServiceNode::timer_callback, this));

    rclcpp::sleep_for(std::chrono::seconds(1));
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      subscription_2_hole_geometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<geometry_msgs::msg::TransformStamped>
      calibrated_msg_base_camera_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_calibrating;
  bool was_calibrated_before;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  void
  service_state_callback(const std::shared_ptr<SetBool::Request> request,
                         const std::shared_ptr<SetBool::Response> response) {
    if (request->data == CALIBRATING) {
      this->is_calibrating = CALIBRATING;
      response->success = true;
      response->message = "successfully change mode to CALIBRATING";
    }
    if (request->data == PUBLISING_CALIBRATED) {
      this->is_calibrating = PUBLISING_CALIBRATED;
      response->success = true;
      response->message = "successfully change mode to PUBLISING_CALIBRATED";
    }

  }

  void hole_geometry_service_callback(
      const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "hole_geometry_service_callback");

    if (this->is_calibrating) { // CALIBRATING

    //------------ broadcast hole_frame TF

    geometry_msgs::msg::TransformStamped msg_hole_camera;
    rclcpp::Time now = this->get_clock()->now();
    msg_hole_camera.header.stamp = now;
    msg_hole_camera.header.frame_id = msg->header.frame_id;
    msg_hole_camera.child_frame_id = msg->child_frame_id;
    msg_hole_camera.transform.translation.x = msg->transform.translation.x;
    msg_hole_camera.transform.translation.y = msg->transform.translation.y;
    msg_hole_camera.transform.translation.z = msg->transform.translation.z;
    msg_hole_camera.transform.rotation.x = msg->transform.rotation.x;
    msg_hole_camera.transform.rotation.y = msg->transform.rotation.y;
    msg_hole_camera.transform.rotation.z = msg->transform.rotation.z;
    msg_hole_camera.transform.rotation.w = msg->transform.rotation.w;
    tf_broadcaster_->sendTransform(msg_hole_camera);
    was_calibrated_before = true;
    
    } else { // PUBLISING_CALIBRATED
            ;//do nothing
    }
  }

  void timer_callback() {
    if(was_calibrated_before){
        try {
            rclcpp::Time now3 = this->get_clock()->now();
            calibrated_msg_base_camera_->header.stamp = now3;
            tf_broadcaster_->sendTransform(*calibrated_msg_base_camera_);
        } catch (...) {
            if (this->is_calibrating)
                RCLCPP_INFO(this->get_logger(), "publishing calibrating TF failed");
            else
                RCLCPP_INFO(this->get_logger(),
                            "publishing previously calibrated TF failed");
            return;
        }
        if (this->is_calibrating)
            RCLCPP_INFO(this->get_logger(), "publishing calibrating TF succeed");
        else
            RCLCPP_INFO(this->get_logger(), "publishing previously calibrated TF succeed");
        
        }

  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto tf2_hole_pub_service_node_std =
      std::make_shared<Tf2PubServiceNode>(node_options);
  rclcpp::spin(tf2_hole_pub_service_node_std);

  rclcpp::shutdown();
  return 0;
}
