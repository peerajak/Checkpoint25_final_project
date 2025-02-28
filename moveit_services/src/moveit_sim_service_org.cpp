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


static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_sim_service");
static const std::string PLANNING_GROUP = "ur_manipulator";


class MoveitSimServerNode : public rclcpp::Node
{
public:
  MoveitSimServerNode(): Node("moveit_sim_service_node")
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    //this->move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    rviz_visual_tools::RvizVisualTools visual_tools_org("base_link", "planning_scene_ros_api_tutorial", this->shared_from_this());//move_group_node);
    visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>(visual_tools_org);
    visual_tools->loadRemoteControl();
    visual_tools->deleteAllMarkers();
    srv_ = create_service<SetBool>("moveit_sim_service", std::bind(&MoveitSimServerNode::moveit_sim_callback, this, _1, _2));
  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  //std::shared_ptr<rclcpp::Node> move_group_node;
  std::shared_ptr<rviz_visual_tools::RvizVisualTools>visual_tools;


  void moveit_sim_callback( const std::shared_ptr<SetBool::Request> request, const std::shared_ptr<SetBool::Response> response) 
    {

     
        if(request->data == DATA_GO_SHOW){
            RCLCPP_INFO(LOGGER, "moveit_sim_callback");
            //visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
            moveit::planning_interface::MoveGroupInterface move_group( this->shared_from_this(),PLANNING_GROUP);
            RCLCPP_INFO(LOGGER, "moveit_sim_callback2");

            const moveit::core::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            RCLCPP_INFO(LOGGER, "moveit_sim_callback3");
    
            RCLCPP_INFO(LOGGER, "Planning frame: %s",move_group.getPlanningFrame().c_str());
    
            RCLCPP_INFO(LOGGER, "End effector link: %s",move_group.getEndEffectorLink().c_str());
    
            RCLCPP_INFO(LOGGER, "Available Planning Groups:");
            std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
    
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
            joint_group_positions[0] = 2.808555; // Shoulder Pan
            joint_group_positions[1] = -1.5708;  // Shoulder Lift
            joint_group_positions[2] = 0.0;      // Elbow
            joint_group_positions[3] = -1.5708;  // Wrist 1
            joint_group_positions[4] = 0.0;      // Wrist 2
            joint_group_positions[5] = 0.0;      // Wrist 3
            move_group.setJointValueTarget(joint_group_positions);
    
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
            bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
            move_group.execute(my_plan);
            sleep(5);
            // step 2
    
            // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            joint_group_positions[0] = 2.808555;   // Shoulder Pan
            joint_group_positions[1] = -0.0697777; // Shoulder Lift
            joint_group_positions[2] = 1.604888;   // Elbow
            joint_group_positions[3] = -1.849111;  // Wrist 1
            joint_group_positions[4] = -0.348888;  // Wrist 2
            joint_group_positions[5] = -1.482777;  // Wrist 3
    
            move_group.setJointValueTarget(joint_group_positions);
            bool success2 = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
            move_group.execute(my_plan);

            if(success && success2){
                response->success = true;
                response->message = "moveit to show: success";
            } else {
                response->success = false;
                response->message = "moveit to show: failed";                
            }

        }
        // if(request->data == DATA_GO_HOME){
        //     response->success = false;
        //     response->message = "moveit to show: failed";                 
        // }


     

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MoveitSimServerNode>());
  rclcpp::executors::SingleThreadedExecutor executor;
  auto move_group_node = std::make_shared<MoveitSimServerNode>();
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  while (true) {
    sleep(1);
  }
  rclcpp::shutdown();
  return 0;
}