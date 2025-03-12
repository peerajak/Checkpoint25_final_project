
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <unistd.h>
#include <cmath>


using namespace std::chrono_literals;

using std::placeholders::_1;


class Tf2Pub : public rclcpp::Node {
    public:
      Tf2Pub() : Node("tf2_pub_node") {

        this->wait_time1 = 1;
        this->wait_time2 = 2;

        callback_group_1 = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_2 = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_3_aruco_geometry = this->create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
    
        timer1_ = this->create_wall_timer(
            500ms, std::bind(&Tf2Pub::timer_callback_1, this), callback_group_1);
        timer2_ = this->create_wall_timer(
            500ms, std::bind(&Tf2Pub::timer_callback_2, this), callback_group_2);


        rclcpp::SubscriptionOptions options2_aruco_geometry;
        options2_aruco_geometry.callback_group = callback_group_3_aruco_geometry;
        subscription_2_aruco_geometry = this->create_subscription<geometry_msgs::msg::Twist>(
            "/aruco_geometry", 10,
            std::bind(&Tf2Pub::aruco_geometry_callback, this,
                      std::placeholders::_1),
            options2_aruco_geometry);
    
      }
    
    private:
    void execute() {
      rclcpp::Rate loop_rate(1);
      RCLCPP_INFO(this->get_logger(),
                  "execute() ");
      while (rclcpp::ok()) {

        loop_rate.sleep();
      }

    }

    void timer_callback_1() {
        sleep(this->wait_time1);
        RCLCPP_INFO(this->get_logger(), "TIMER CALLBACK 1");
        timer1_->cancel();
        //assert(false);
        std::thread{std::bind(&Tf2Pub::execute, this)}.detach();
      }
    
      void timer_callback_2() {
        sleep(this->wait_time2);
        RCLCPP_INFO(this->get_logger(), "TIMER CALLBACK 2");
      }
      rclcpp::CallbackGroup::SharedPtr callback_group_1;
      rclcpp::CallbackGroup::SharedPtr callback_group_2;  
      rclcpp::CallbackGroup::SharedPtr callback_group_3_aruco_geometry;
      rclcpp::TimerBase::SharedPtr timer1_;
      rclcpp::TimerBase::SharedPtr timer2_;
      float wait_time1;
      float wait_time2;
    };

    void aruco_geometry_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    }

    int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);


        std::shared_ptr<Tf2Pub> tf2_pub_node =
            std::make_shared<Tf2Pub>();
      
        rclcpp::Node::make_shared("executor_example_5_node");
      
        // This is the same as a print in ROS
        RCLCPP_INFO(tf2_pub_node->get_logger(), "timer_1_node INFO...");
      
        // Same code, but in steps
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(tf2_pub_node);
        executor.spin();
      

        rclcpp::shutdown();
        return 0;
      }
