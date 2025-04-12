#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <cmath>
#include <unistd.h>


#define CALIBRATING true
#define PUBLISING_CALIBRATED false
#define SLEEPTIME 3
using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;



static const rclcpp::Logger LOGGER = rclcpp::get_logger("tf2_pub_service_node");


class Tf2PubServiceNode : public rclcpp::Node {
public:
  Tf2PubServiceNode(rclcpp::NodeOptions &node_options)
      : Node("tf2_pub_service_node", node_options) {
    this->is_calibrating = CALIBRATING;
    node_ = std::make_shared<rclcpp::Node>("example_group_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    srv_ = create_service<SetBool>(
        "tf2_pub_service",
        std::bind(&Tf2PubServiceNode::service_state_callback, this, _1, _2));
    calibrated_msg_base_camera_ =  std::make_shared<geometry_msgs::msg::TransformStamped>();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subscription_2_aruco_geometry =
        this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "aruco_point_wrt_camera", 10,
            std::bind(&Tf2PubServiceNode::aruco_geometry_service_callback, this, _1));

      timer_ = this->create_wall_timer(
      500ms, std::bind(&Tf2PubServiceNode::timer_callback, this));

    rclcpp::sleep_for(std::chrono::seconds(1));
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
      subscription_2_aruco_geometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<geometry_msgs::msg::TransformStamped> calibrated_msg_base_camera_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool is_calibrating;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

    void service_state_callback(const std::shared_ptr<SetBool::Request> request,
                        const std::shared_ptr<SetBool::Response> response) {
     if(request->data == CALIBRATING){
        this->is_calibrating = CALIBRATING;
     }
     if(request->data == PUBLISING_CALIBRATED){
        this->is_calibrating = PUBLISING_CALIBRATED;
     }     
     response->success = true;
     response->message = "succeed";                  
                        
    }

  void aruco_geometry_service_callback(
      const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "aruco_geometry_service_callback");
    if( this->is_calibrating) { //CALIBRATING
        rclcpp::Time now = this->get_clock()->now();
        std::string fromFrame = "aruco_frame";
        std::string toFrame = "base_link";

        geometry_msgs::msg::TransformStamped tf_aruco_to_base_link;
        try {
        tf_aruco_to_base_link =
            tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    toFrame.c_str(), fromFrame.c_str(), ex.what());
        return;
        }

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

        // This q, and p together form a Pose. This is aruco w.r.t camera Pose.
        tf2::Quaternion q_aruco_camera(
            msg->transform.rotation.x, msg->transform.rotation.y,
            msg->transform.rotation.z, msg->transform.rotation.w);
        tf2::Vector3 p_aruco_camera(msg->transform.translation.x,
                                    msg->transform.translation.y,
                                    msg->transform.translation.z);

        tf2::Transform transform_aruco_camera(q_aruco_camera, p_aruco_camera);
        tf2::Vector3 p_camera_aruco = transform_aruco_camera.inverse().getOrigin();
        tf2::Quaternion q_camera_aruco =
            transform_aruco_camera.inverse().getRotation();


        tf2::Vector3 p_base_camera = transform_aruco_to_base * p_camera_aruco;
        tf2::Quaternion q_base_camera = transform_aruco_to_base * q_camera_aruco;


        //------------ broadcast TF


        std::string fromFrameRel = "base_link";
        std::string toFrameRel =
            "fake_camera_frame_from_base"; // TODO change Camera link name
        //geometry_msgs::msg::TransformStamped msg_base_camera;
        rclcpp::Time now2 = this->get_clock()->now();
        calibrated_msg_base_camera_->header.stamp = now2;
        calibrated_msg_base_camera_->header.frame_id = fromFrameRel;
        calibrated_msg_base_camera_->child_frame_id = toFrameRel;
        calibrated_msg_base_camera_->transform.translation.x = p_base_camera.getX();
        calibrated_msg_base_camera_->transform.translation.y = p_base_camera.getY();
        calibrated_msg_base_camera_->transform.translation.z = p_base_camera.getZ();
        calibrated_msg_base_camera_->transform.rotation.x = q_base_camera.getX();
        calibrated_msg_base_camera_->transform.rotation.y = q_base_camera.getY();
        calibrated_msg_base_camera_->transform.rotation.z = q_base_camera.getZ();
        calibrated_msg_base_camera_->transform.rotation.w = q_base_camera.getW();
        tf_broadcaster_->sendTransform(*calibrated_msg_base_camera_);

    }else{//PUBLISING_CALIBRATED

     ;// do nothing

    }
 }


    void timer_callback()
    {
        try{
            rclcpp::Time now3 = this->get_clock()->now();
            calibrated_msg_base_camera_->header.stamp = now3;
            tf_broadcaster_->sendTransform(*calibrated_msg_base_camera_);
        } catch(...) {
            if (this->is_calibrating)
            RCLCPP_INFO(this->get_logger(), "publishing calibrating TF failed");
            else RCLCPP_INFO(this->get_logger(), "publishing previously calibrated TF failed");

            return;    
        }
        if(this->is_calibrating)
        RCLCPP_INFO(this->get_logger(), "publishing calibrating TF succeed");
        else RCLCPP_INFO(this->get_logger(), "publishing previously calibrated TF succeed");
    }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto tf2_pub_service_node_std =
      std::make_shared<Tf2PubServiceNode>(node_options);
  rclcpp::spin(tf2_pub_service_node_std);

  rclcpp::shutdown();
  return 0;
}
