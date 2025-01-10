#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP = "ur_manipulator";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // planning scene ros api
  //  Visualization
  //  ^^^^^^^^^^^^^
  //  The package MoveItVisualTools provides many capabilities for visualizing
  //  objects, robots, and trajectories in RViz as well as debugging tools such
  //  as step-by-step introspection of a script.
  rviz_visual_tools::RvizVisualTools visual_tools(
      "base_link", "planning_scene_ros_api_tutorial", move_group_node);
  visual_tools.loadRemoteControl();
  visual_tools.deleteAllMarkers();

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr
      planning_scene_diff_publisher =
          move_group_node->create_publisher<moveit_msgs::msg::PlanningScene>(
              "planning_scene", 1);

  while (planning_scene_diff_publisher->get_subscription_count() < 1) {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Define the attached object message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We will use this message to add or
  // subtract the object from the world
  // and to attach the object to the robot.
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link"; // panda_hand
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "base_link";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.11;

  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.275;

  // Add the shape and pose to the collision object
  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation.
  attached_object.object.operation = attached_object.object.ADD;

  // Since we are attaching the object to the robot hand to simulate picking up
  // the object, we want the collision checker to ignore collisions between the
  // object and the robot hand.
  //attached_object.touch_links = std::vector<std::string>{
  //    "panda_hand", "panda_leftfinger", "panda_rightfinger"};

  // Add an object into the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Add the object into the environment by adding it to
  // the set of collision objects in the "world" part of the
  // planning scene. Note that we are using only the "object"
  // field of the attached_object message here.
  RCLCPP_INFO(LOGGER,
              "Adding the object into the world at the location of the hand.");
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Interlude: Synchronous vs Asynchronous updates
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // There are two separate mechanisms available to interact
  // with the move_group node using diffs:
  //
  // * Send a diff via a rosservice call and block until
  //   the diff is applied (synchronous update)
  // * Send a diff via a topic, continue even though the diff
  //   might not be applied yet (asynchronous update)
  //
  // While most of this tutorial uses the latter mechanism (given the long
  // sleeps inserted for visualization purposes asynchronous updates do not pose
  // a problem), it would be perfectly justified to replace the
  // planning_scene_diff_publisher by the following service client:
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr
      planning_scene_diff_client =
          move_group_node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
              "apply_planning_scene");
  planning_scene_diff_client->wait_for_service();
  // and send the diffs to the planning scene via a service call:
  auto request =
      std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;
  std::shared_future<
      std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>>
      response_future;
  response_future = planning_scene_diff_client->async_send_request(request);

  // wait for the service to respond
  std::chrono::seconds wait_time(1);
  std::future_status fs = response_future.wait_for(wait_time);
  if (fs == std::future_status::timeout) {
    RCLCPP_ERROR(LOGGER, "Service timed out.");
  } else {
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>
        planning_response;
    planning_response = response_future.get();
    if (planning_response->success) {
      RCLCPP_INFO(LOGGER, "Service successfully added object.");
    } else {
      RCLCPP_ERROR(LOGGER, "Service failed to add object.");
    }
  }
  // end planning scene

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node,
                                                            PLANNING_GROUP);

  const moveit::core::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(LOGGER, "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,
                                         joint_group_positions);

  // joint_group_positions[0] = 0.00;  // Shoulder Pan
  joint_group_positions[1] = -2.50; // Shoulder Lift
  joint_group_positions[2] = 1.50;  // Elbow
  joint_group_positions[3] = -1.50; // Wrist 1
  joint_group_positions[4] = -1.55; // Wrist 2
  // joint_group_positions[5] = 0.00;  // Wrist 3
  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  move_group.execute(my_plan);

  rclcpp::shutdown();
  return 0;
}