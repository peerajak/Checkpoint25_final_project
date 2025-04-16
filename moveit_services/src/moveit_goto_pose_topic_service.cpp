#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include "cp25_custom_interfaces/srv/pose_to_moveit.hpp"  
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;
// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_goto_pose_topic_service");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

/*
Service name moveit_goto_pose_service type  with call back at moveit_goto_pose_callback
*/

class MoveitGotoPoseServerNode : public rclcpp::Node
{
public:
  MoveitGotoPoseServerNode(rclcpp::NodeOptions &node_options): Node("moveit_goto_pose_service_node", node_options)
  {


    node_ = std::make_shared<rclcpp::Node>("moveit_goto_pose_service");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    srv_ = create_service<cp25_custom_interfaces::srv::PoseToMoveit>("moveit_goto_pose_service", std::bind(&MoveitGotoPoseServerNode::moveit_goto_pose_callback, this, _1, _2));
   


    rclcpp::sleep_for(std::chrono::seconds(1));
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });

   // initialize move_group interfaces
    move_group_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, PLANNING_GROUP_ROBOT);

    // get initial state of robot
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);

    // print out basic system information

    move_group_robot_->setPoseReferenceFrame("base_link");
    RCLCPP_INFO(LOGGER, "set Pose Reference Frame to: %s",
                move_group_robot_->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
                move_group_robot_->setEndEffectorLink("rg2_gripper_aruco_link");
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    // set start state of robot to current state
    move_group_robot_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Goal Pose Trajectory");

  }




private:
  rclcpp::Service<cp25_custom_interfaces::srv::PoseToMoveit>::SharedPtr srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_robot_;
  //std::shared_ptr<rviz_visual_tools::RvizVisualTools>visual_tools;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;


  const moveit::core::JointModelGroup *joint_model_group_robot_;

  // declare trajectory planning variables for robot
  std::vector<double> joint_group_positions_robot_;
  moveit::core::RobotStatePtr current_state_robot_;
  moveit::planning_interface::MoveGroupInterface::Plan kinematics_trajectory_plan_;
  geometry_msgs::msg::Pose target_pose_robot_;
  bool plan_success_robot_ = false;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;
  std::vector<geometry_msgs::msg::Pose> cartesian_waypoints_;
  double plan_fraction_robot_ = 0.0;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;

  //rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;


//   void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
//   {
//     RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->position.x);
//   }


  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }


  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(this->kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  float how_far_from_target(float target_pos_x, float target_pos_y, float target_pos_z){
    return sqrt((move_group_robot_->getCurrentPose().pose.position.x - target_pos_x )*(move_group_robot_->getCurrentPose().pose.position.x - target_pos_x ) + 
           (move_group_robot_->getCurrentPose().pose.position.y - target_pos_y )*(move_group_robot_->getCurrentPose().pose.position.y - target_pos_y ) +
           (move_group_robot_->getCurrentPose().pose.position.z - target_pos_z )* (move_group_robot_->getCurrentPose().pose.position.z - target_pos_z ));
  }
  void moveit_goto_pose_callback( const std::shared_ptr<cp25_custom_interfaces::srv::PoseToMoveit::Request> request, const std::shared_ptr<cp25_custom_interfaces::srv::PoseToMoveit::Response> response) 
    {
        float far_threshold = 0.2; //more than 20 cm is far
        float small_movement = 0.01;
        if(how_far_from_target(request->x, request->y, request->z) > far_threshold){ //is far away
            RCLCPP_INFO(LOGGER, "Far movement");
            setup_goal_pose_target(request->x, request->y, request->z, request->ax, request->ay, request->az, request->aw);
            //setup_goal_pose_target(+0.343, +0.132, +0.284, -1.000, +0.000, +0.000, +0.000);
            plan_trajectory_kinematics();
            move_group_robot_->execute(this->kinematics_trajectory_plan_);
            response->success = true;
            response->description = "success";      
        }else{
            RCLCPP_INFO(LOGGER, "Near movement");
            // 1. X direction: find the different in x direction, the sign of the different, use a small movement to adjust
            // 2. X direction: loop while setup_waypoint from current position until the target. use target value for the last loop
            //  inside the loop, do setup_waypoint
            // 3. X direction:     plan_trajectory_cartesian() and  execute_trajectory_cartesian();
            // 4. Repeat 1-3 for Y and Z directions
            float x_diff = request->x - move_group_robot_->getCurrentPose().pose.position.x ;
            float y_diff = request->y - move_group_robot_->getCurrentPose().pose.position.y;
            float z_diff = request->z - move_group_robot_->getCurrentPose().pose.position.z;
            float x_planing =  move_group_robot_->getCurrentPose().pose.position.x ;
            float y_planing =  move_group_robot_->getCurrentPose().pose.position.y ;
            float z_planing =  move_group_robot_->getCurrentPose().pose.position.z ;
            while(std::abs(x_diff) > small_movement){
                if (x_diff > 0){
                    if (request->x > x_planing + small_movement){
                        x_planing += small_movement;
                        x_diff =  request->x - x_planing ;
                    }                            
                }else if(x_diff > 0){
                    if (request->x < x_planing - small_movement){
                        x_planing -= small_movement;
                    }     
                } 
                setup_waypoints_target(x_planing,y_planing,z_planing);          
            }

            while(std::abs(y_diff) > small_movement){
                if (y_diff > 0){
                    if (request->y > y_planing + small_movement){
                        y_planing += small_movement;
                        y_diff =  request->y - y_planing ;
                    }                            
                }else if(y_diff > 0){
                    if (request->y < y_planing - small_movement){
                        y_planing -= small_movement;
                    }     
                } 
                setup_waypoints_target(x_planing,y_planing,z_planing);          
            }

            while(std::abs(z_diff) > small_movement){
                if (z_diff > 0){
                    if (request->z > z_planing + small_movement){
                        z_planing += small_movement;
                        z_diff =  request->z - z_planing ;
                    }                            
                }else if(z_diff > 0){
                    if (request->z < z_planing - small_movement){
                        z_planing -= small_movement;
                    }     
                } 
                setup_waypoints_target(x_planing,y_planing,z_planing);          
            }

            plan_trajectory_cartesian();
            RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
            execute_trajectory_cartesian();
            response->success = true;
            response->description = "success";   

        }

    }

};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node_std = std::make_shared<MoveitGotoPoseServerNode>(node_options);
  rclcpp::spin(move_group_node_std );

  rclcpp::shutdown();
  return 0;
}

