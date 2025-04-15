#include "rclcpp/rclcpp.hpp"
#include "cp25_custom_interfaces/srv/pose_to_moveit.hpp"                                     // CHANGE
#include "geometry_msgs/msg/pose.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;


class MoveitGotoPoseTopicServerServiceClient : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  MoveitGotoPoseTopicServerServiceClient()
  : Node("moveit_goto_pose_topic_server_service_client")
  {
    client_ = this->create_client<cp25_custom_interfaces::srv::PoseToMoveit>("moveit_goto_pose_service");   
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "moveit_goto_pose", 10, std::bind(&MoveitGotoPoseTopicServerServiceClient::topic_callback, this, _1));
    // timer_ = this->create_wall_timer(
    //     1s, std::bind(&MoveitGotoPoseTopicServerServiceClient::timer_callback, this));
  }
  bool is_service_done() const { return this->service_done_; }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  rclcpp::Client<cp25_custom_interfaces::srv::PoseToMoveit>::SharedPtr client_;   
  //rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_called_ = false;     
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 
  void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    // Print the value 'data' inside the 'msg' parameter
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->position.x);

    std::shared_ptr<cp25_custom_interfaces::srv::PoseToMoveit::Request> request = 
        std::make_shared<cp25_custom_interfaces::srv::PoseToMoveit::Request>();  
    request->x = msg->position.x;
    request->y = msg->position.y;
    request->z = msg->position.z;
    request->ax = msg->orientation.x;
    request->ay = msg->orientation.y;
    request->az = msg->orientation.z;
    request->aw = msg->orientation.w;
                                                        
    if (!service_called_) {
        RCLCPP_INFO(this->get_logger(), "Send Async Request");
        while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Client interrupted while waiting for service. Terminating...");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Service Unavailable. Waiting for Service...");
        }
        auto result_future = client_->async_send_request(
            request, std::bind(&MoveitGotoPoseTopicServerServiceClient::response_callback, this,
                            std::placeholders::_1));
        service_called_ = true;

        // Now check for the response after a timeout of 1 second
        auto status = result_future.wait_for(1s);

        if (status != std::future_status::ready) {

        RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
        }
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
    }

  }



  void
  response_callback(rclcpp::Client<cp25_custom_interfaces::srv::PoseToMoveit>::SharedFuture future) {

    // Get response value
    auto response = future.get();
    if(response->success){
        RCLCPP_INFO(this->get_logger(), "Response: success");
        service_done_ = true;  
    }else{
        RCLCPP_INFO(this->get_logger(), "Response: failed");
        service_done_ = true; 
    }

  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveitGotoPoseTopicServerServiceClient>());
  rclcpp::shutdown();
  return 0;
}





