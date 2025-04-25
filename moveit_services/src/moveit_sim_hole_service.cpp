#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


#define DATA_GO_SHOW true
#define DATA_GO_HOME false
#define SLEEPTIME 4
using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_sim_hole_service");
static const std::string PLANNING_GROUP = "ur_manipulator";


class MoveitSimServerNode : public rclcpp::Node
{
public:
  MoveitSimServerNode(rclcpp::NodeOptions &node_options): Node("moveit_sim_hole_service_node", node_options)
  {


    node_ = std::make_shared<rclcpp::Node>("moveit_sim_hole_service");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    srv_ = create_service<SetBool>("moveit_sim_hole_service", std::bind(&MoveitSimServerNode::moveit_sim_hole_callback, this, _1, _2));
   
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,PLANNING_GROUP);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // rviz_visual_tools::RvizVisualTools visual_tools_org("base_link", "planning_scene_ros_api_tutorial", node_);
    // visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(visual_tools_org);
    // visual_tools->loadRemoteControl();
    // visual_tools->deleteAllMarkers();
    rclcpp::sleep_for(std::chrono::seconds(1));
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });

  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  //std::shared_ptr<rviz_visual_tools::RvizVisualTools>visual_tools;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::Pose target_pose_robot_;
  std::vector<geometry_msgs::msg::Pose> cartesian_waypoints_;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;
  double plan_fraction_robot_ = 0.0;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;

  void setup_waypoints_target(float target_x, float target_y, float target_z) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis

    double xyz_resolution = 0.0001, xy_goal_threshold = 0.0001, z_goal_threshold = 0.01;


    while (abs(target_pose_robot_.position.x - target_x) > xy_goal_threshold) {
    target_pose_robot_.position.x += xyz_resolution *
                                (target_x - target_pose_robot_.position.x) /
                                abs(target_pose_robot_.position.x - target_x);
    cartesian_waypoints_.push_back(target_pose_robot_);
    }
    while (abs(target_pose_robot_.position.y - target_y) > xy_goal_threshold) {
    target_pose_robot_.position.y += xyz_resolution *
                                (target_y - target_pose_robot_.position.y) /
                                abs(target_pose_robot_.position.y - target_y);
    cartesian_waypoints_.push_back(target_pose_robot_);
    }
    while (abs(target_pose_robot_.position.z - target_z) > z_goal_threshold) {
    target_pose_robot_.position.z += xyz_resolution *
                                (target_z - target_pose_robot_.position.z) /
                                abs(target_pose_robot_.position.z - target_z);
    cartesian_waypoints_.push_back(target_pose_robot_);
    }


    // target_pose_robot_.position.x += x_delta;
    // target_pose_robot_.position.y += y_delta;
    // target_pose_robot_.position.z += z_delta;
    // // add the desired pose to the target waypoints vector
    // cartesian_waypoints_.push_back(target_pose_robot_);
    // RCLCPP_INFO(LOGGER, "setup_waypoints_target(%f, %f, %f)",x_delta, y_delta, z_delta);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  float how_far_from_target(float delta_x, float delta_y, float delta_z){
    return sqrt( delta_x * delta_x  + delta_y * delta_y + delta_z * delta_z );
  }


  void moveit_sim_hole_callback( const std::shared_ptr<SetBool::Request> request, const std::shared_ptr<SetBool::Response> response) 
    {

        RCLCPP_INFO(LOGGER, "moveit_sim_hole_callback");
        float far_threshold = 0.2; //more than 20 cm is far
        RCLCPP_INFO(LOGGER, "Executing Hole Cartesian Trajectory...");
        std::string fromFrame = "base_link";  // parent
        std::string toFrame = "hole_frame"; // child
        float z_offset = 0.25;//hard code this is the distance from tool0 to from end_effector_tip_link 

        geometry_msgs::msg::TransformStamped tf_hole_to_base_link;
        try {
            tf_hole_to_base_link =
                tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                        toFrame.c_str(), fromFrame.c_str(), ex.what());
            return;
        } // checked passed
        float x_diff = tf_hole_to_base_link.transform.translation.x - move_group->getCurrentPose().pose.position.x ;
        float y_diff = tf_hole_to_base_link.transform.translation.y - move_group->getCurrentPose().pose.position.y;
        float z_diff = tf_hole_to_base_link.transform.translation.z - move_group->getCurrentPose().pose.position.z + z_offset;

        if(how_far_from_target(x_diff, y_diff, z_diff) > far_threshold){ //is far away
            RCLCPP_INFO(LOGGER, "Far movement");
            const moveit::core::JointModelGroup *joint_model_group =
            move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);


            move_group->setPoseReferenceFrame("base_link");
            RCLCPP_INFO(LOGGER, "set Pose Reference Frame to: %s",
                        move_group->getPoseReferenceFrame().c_str());


    
            RCLCPP_INFO(LOGGER, "Planning frame: %s",move_group->getPlanningFrame().c_str());

            move_group->setEndEffectorLink("tool0");    
            RCLCPP_INFO(LOGGER, "End effector link: %s",move_group->getEndEffectorLink().c_str());
    
            RCLCPP_INFO(LOGGER, "Available Planning Groups:");
            std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    
            moveit::core::RobotStatePtr current_state = move_group->getCurrentState(10);
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
            joint_group_positions[0] = -0.01745; // Shoulder Pan
            joint_group_positions[1] = -3.7;  // Shoulder Lift
            joint_group_positions[2] = -0.418879;      // Elbow
            joint_group_positions[3] = -0.69813;  // Wrist 1
            joint_group_positions[4] = 1.6057;      // Wrist 2
            joint_group_positions[5] = -0.174532;      // Wrist 3
            move_group->setJointValueTarget(joint_group_positions);
    
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
            bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
            move_group->execute(my_plan);
            sleep(SLEEPTIME);
        }
            // step 2  no longer far
            
            
            RCLCPP_INFO(LOGGER, "Finetune movement");
            setup_waypoints_target(tf_hole_to_base_link.transform.translation.x ,
            tf_hole_to_base_link.transform.translation.y, tf_hole_to_base_link.transform.translation.z+ z_offset);   
            plan_trajectory_cartesian();

            execute_trajectory_cartesian();
            sleep(SLEEPTIME);
            response->success = true;
            response->message = "moveit to hole: success";

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node_std = std::make_shared<MoveitSimServerNode>(node_options);
  rclcpp::spin(move_group_node_std );

  rclcpp::shutdown();
  return 0;
}

