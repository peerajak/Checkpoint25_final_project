#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>


#define DATA_GO_SHOW true
#define DATA_GO_HOME false
using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_sim_service");
static const std::string PLANNING_GROUP = "ur_manipulator";
moveit::planning_interface::MoveGroupInterface *move_group_global_ptr;




  void moveit_sim_callback( const std::shared_ptr<SetBool::Request> request, const std::shared_ptr<SetBool::Response> response) 
    {
     
        if(request->data == DATA_GO_SHOW){
            RCLCPP_INFO(LOGGER, "moveit_sim_callback");
            //visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
            
            RCLCPP_INFO(LOGGER, "moveit_sim_callback2");

            const moveit::core::JointModelGroup *joint_model_group =
            move_group_global_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            RCLCPP_INFO(LOGGER, "moveit_sim_callback3");
    
            RCLCPP_INFO(LOGGER, "Planning frame: %s",move_group_global_ptr->getPlanningFrame().c_str());
    
            RCLCPP_INFO(LOGGER, "End effector link: %s",move_group_global_ptr->getEndEffectorLink().c_str());
    
            RCLCPP_INFO(LOGGER, "Available Planning Groups:");
            std::copy(move_group_global_ptr->getJointModelGroupNames().begin(), move_group_global_ptr->getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    
            moveit::core::RobotStatePtr current_state = move_group_global_ptr->getCurrentState(10);
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
            joint_group_positions[0] = 2.808555; // Shoulder Pan
            joint_group_positions[1] = -1.5708;  // Shoulder Lift
            joint_group_positions[2] = 0.0;      // Elbow
            joint_group_positions[3] = -1.5708;  // Wrist 1
            joint_group_positions[4] = 0.0;      // Wrist 2
            joint_group_positions[5] = 0.0;      // Wrist 3
            move_group_global_ptr->setJointValueTarget(joint_group_positions);
    
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
            bool success = (move_group_global_ptr->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
            move_group_global_ptr->execute(my_plan);
            sleep(5);
            // step 2
    
            // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            joint_group_positions[0] = 2.808555;   // Shoulder Pan
            joint_group_positions[1] = -0.0697777; // Shoulder Lift
            joint_group_positions[2] = 1.604888;   // Elbow
            joint_group_positions[3] = -1.849111;  // Wrist 1
            joint_group_positions[4] = -0.348888;  // Wrist 2
            joint_group_positions[5] = -1.482777;  // Wrist 3
    
            move_group_global_ptr->setJointValueTarget(joint_group_positions);
            bool success2 = (move_group_global_ptr->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
            move_group_global_ptr->execute(my_plan);

            if(success && success2){
                response->success = true;
                response->message = "moveit to show: success";
            } else {
                response->success = false;
                response->message = "moveit to show: failed";                
            }

        }
        if(request->data == DATA_GO_HOME){
            response->success = false;
            response->message = "moveit to show: failed";                 
        }
};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MoveitSimServerNode>());
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_sim_service_node", node_options);
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  moveit::planning_interface::MoveGroupInterface move_group( move_group_node,PLANNING_GROUP);
  move_group_global_ptr = &move_group;
  rviz_visual_tools::RvizVisualTools visual_tools("base_link", "planning_scene_ros_api_tutorial", move_group_node);
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();
  rclcpp::Service<SetBool>::SharedPtr srv_ = move_group_node->create_service<SetBool>("moveit_sim_service", std::bind(&moveit_sim_callback,_1, _2 ));


  while (true) {
    sleep(1);
  }
  rclcpp::shutdown();
  return 0;
}