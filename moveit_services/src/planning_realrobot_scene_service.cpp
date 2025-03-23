#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <memory>

using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_service");
static const std::string PLANNING_GROUP = "ur_manipulator";



class PlanningSceneServerNode : public rclcpp::Node
{
public:
  PlanningSceneServerNode()
  : Node("planning_scene_service")
  {

    srv_ = create_service<Empty>("planning_scene_cp25", std::bind(&PlanningSceneServerNode::planning_scene_callback, this, _1, _2));
    planning_scene_diff_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
    planning_scene_diff_client = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");

  }

private:
  rclcpp::Service<Empty>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client;

  void planning_scene_callback(
      const std::shared_ptr<Empty::Request> request,
      const std::shared_ptr<Empty::Response>
          response) 
    {

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "base_link";
    attached_object.object.header.frame_id = "base_link";
    attached_object.object.id = "box";

    geometry_msgs::msg::Pose pose_counter, pose_top, pose_wall;
    geometry_msgs::msg::Pose pose_coffee_machine_box;
    pose_counter.position.x = 0.3;
    pose_counter.position.y = 0.36;
    pose_counter.position.z = -0.532;

    pose_counter.orientation.x = 0.0;
    pose_counter.orientation.y = 0.0;
    pose_counter.orientation.z = 0.0;
    pose_counter.orientation.w = 0.0;

    pose_top.position.x = 0.3;
    pose_top.position.y = 0.36;
    pose_top.position.z = -0.032;

    pose_top.orientation.x = 0.0;
    pose_top.orientation.y = 0.0;
    pose_top.orientation.z = 0.0;
    pose_top.orientation.w = 0.0;

    pose_wall.position.x = 0.3;
    pose_wall.position.y = -0.44;
    pose_wall.position.z = -0.032;

    pose_wall.orientation.x = 0.0;
    pose_wall.orientation.y = 0.0;
    pose_wall.orientation.z = 0.0;
    pose_wall.orientation.w = 0.0;

    pose_coffee_machine_box.position.x = 0.1+0.1;
    pose_coffee_machine_box.position.y = 0.86;
    pose_coffee_machine_box.position.z = 0.068+0.509494/2;

    pose_coffee_machine_box.orientation.x = 0.0;
    pose_coffee_machine_box.orientation.y = 0.0;
    pose_coffee_machine_box.orientation.z = 0.0;
    pose_coffee_machine_box.orientation.w = 0.0;

    shape_msgs::msg::SolidPrimitive primitive_counter, primitive_top,
        primitive_wall;

    shape_msgs::msg::SolidPrimitive primitive_coffee_machine_box;
    primitive_counter.type = primitive_counter.BOX;
    primitive_counter.dimensions.resize(3);
    primitive_counter.dimensions[0] = 0.5;
    primitive_counter.dimensions[1] = 1.8;
    primitive_counter.dimensions[2] = 1.0;

    primitive_top.type = primitive_top.BOX;
    primitive_top.dimensions.resize(3);
    primitive_top.dimensions[0] = 0.85;
    primitive_top.dimensions[1] = 1.81;
    primitive_top.dimensions[2] = 0.05;

    primitive_wall.type = primitive_wall.BOX;
    primitive_wall.dimensions.resize(3);
    primitive_wall.dimensions[0] = 2.0;
    primitive_wall.dimensions[1] = 0.3;
    primitive_wall.dimensions[2] = 2.0;

    primitive_coffee_machine_box.type = primitive_coffee_machine_box.BOX;
    primitive_coffee_machine_box.dimensions.resize(3);
    primitive_coffee_machine_box.dimensions[0] = 0.56046;
    primitive_coffee_machine_box.dimensions[1] = 0.212366;
    primitive_coffee_machine_box.dimensions[2] = 0.509494;

    attached_object.object.primitives.push_back(primitive_counter);
    attached_object.object.primitives.push_back(primitive_top);
    attached_object.object.primitives.push_back(primitive_wall);
    attached_object.object.primitive_poses.push_back(pose_counter);
    attached_object.object.primitive_poses.push_back(pose_top);
    attached_object.object.primitive_poses.push_back(pose_wall);

    attached_object.object.primitives.push_back(primitive_coffee_machine_box);
    attached_object.object.primitive_poses.push_back(pose_coffee_machine_box);

    attached_object.object.operation = attached_object.object.ADD;


    RCLCPP_INFO(LOGGER,
                "Adding the object into the world at the location of the hand.");
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
 
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningSceneServerNode>());
  rclcpp::shutdown();
  return 0;
}