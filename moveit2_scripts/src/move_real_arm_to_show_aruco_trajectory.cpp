#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "shape_msgs/msg/detail/mesh__struct.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <memory>

#define PLANNING_SCENE
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
#ifdef PLANNING_SCENE
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr
      planning_scene_diff_publisher =
          move_group_node->create_publisher<moveit_msgs::msg::PlanningScene>(
              "planning_scene", 1);

  while (planning_scene_diff_publisher->get_subscription_count() < 1) {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  //visual_tools.prompt(
  //    "Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Define the attached object message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We will use this message to add or
  // subtract the object from the world
  // and to attach the object to the robot.
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "base_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "base_link";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::msg::Pose pose_counter, pose_top, pose_wall;
  geometry_msgs::msg::Pose pose_coffee_machine_mesh;
  //geometry_msgs::msg::Pose pose_coffee_machine_box;
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
  pose_wall.position.y = -0.64;
  pose_wall.position.z = -0.032;

  pose_wall.orientation.x = 0.0;
  pose_wall.orientation.y = 0.0;
  pose_wall.orientation.z = 0.0;
  pose_wall.orientation.w = 0.0;

  pose_coffee_machine_mesh.position.x = 0.1;
  pose_coffee_machine_mesh.position.y = 0.86;
  pose_coffee_machine_mesh.position.z = -0.032;

  pose_coffee_machine_mesh.orientation.x = 0.0;
  pose_coffee_machine_mesh.orientation.y = 0.0;
  pose_coffee_machine_mesh.orientation.z = 0.707;
  pose_coffee_machine_mesh.orientation.w = 0.707;

//   pose_coffee_machine_box.position.x = 0.1;
//   pose_coffee_machine_box.position.y = 0.86;
//   pose_coffee_machine_box.position.z = -0.032;

//   pose_coffee_machine_box.orientation.x = 0.0;
//   pose_coffee_machine_box.orientation.y = 0.0;
//   pose_coffee_machine_box.orientation.z = 0.707;
//   pose_coffee_machine_box.orientation.w = 0.707;

  /* Define a box to be attached */
  shape_msgs::msg::SolidPrimitive primitive_counter, primitive_top,
      primitive_wall;
  shape_msgs::msg::Mesh coffee_machine_mesh;
  //shape_msgs::msg::SolidPrimitive primitive_coffee_machine_box;
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

//   primitive_coffee_machine_box.type = primitive_coffee_machine_box.BOX;
//   primitive_coffee_machine_box.dimensions.resize(3);
//   primitive_coffee_machine_box.dimensions[0] = 0.212366;
//   primitive_coffee_machine_box.dimensions[1] = 0.36046;
//   primitive_coffee_machine_box.dimensions[2] = 0.409494;

  std::unique_ptr<shapes::Mesh> coffee_machine_shape_mesh_ptr( shapes::createMeshFromResource(
     "package://the_construct_office_gazebo/models/coffee_machine/meshes/"
     "cafeteria.dae"));
  shapes::ShapeMsg shelf_mesh_msg;
  shapes::constructMsgFromShape(coffee_machine_shape_mesh_ptr.get(), shelf_mesh_msg);
  coffee_machine_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

  // Add the shape and pose to the collision object
  attached_object.object.primitives.push_back(primitive_counter);
  attached_object.object.primitives.push_back(primitive_top);
  attached_object.object.primitives.push_back(primitive_wall);
  attached_object.object.primitive_poses.push_back(pose_counter);
  attached_object.object.primitive_poses.push_back(pose_top);
  attached_object.object.primitive_poses.push_back(pose_wall);


  attached_object.object.meshes.push_back(coffee_machine_mesh);
  attached_object.object.mesh_poses.push_back(pose_coffee_machine_mesh);
  //attached_object.object.primitives.push_back(primitive_coffee_machine_box);
  //attached_object.object.primitive_poses.push_back(pose_coffee_machine_box);
  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation.
  attached_object.object.operation = attached_object.object.ADD;


  

  // Since we are attaching the object to the robot hand to simulate picking up
  // the object, we want the collision checker to ignore collisions between the
  // object and the robot hand.
  // attached_object.touch_links = std::vector<std::string>{
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
  //visual_tools.prompt(
  //    "Press 'next' in the RvizVisualToolsGui window to continue the demo");

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
#endif
   visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");
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
 
  joint_group_positions[0] = 2.094395;// Shoulder Pan
  joint_group_positions[1] = -1.5708; // Shoulder Lift
  joint_group_positions[2] = 0.0;   // Elbow
  joint_group_positions[3] = -1.5708; // Wrist 1
  joint_group_positions[4] = 0.0; // Wrist 2
  joint_group_positions[5] = 0.0; // Wrist 3
  move_group.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success =
      (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  move_group.execute(my_plan);
   visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // step 2

   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   joint_group_positions[0] = 2.094395;// Shoulder Pan
   joint_group_positions[1] = -0.0697777; // Shoulder Lift
   joint_group_positions[2] = 1.604888;   // Elbow
   joint_group_positions[3] = -1.849111; // Wrist 1
   joint_group_positions[4] = -0.348888; // Wrist 2
   joint_group_positions[5] = -1.482777;// Wrist 3
   

   move_group.setJointValueTarget(joint_group_positions);
   success =   (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

   move_group.execute(my_plan);


  /*
  // step 3
   visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // print current pose
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose desired_pose = current_pose ;
    // Print the current pose of the end effector
    RCLCPP_INFO(LOGGER, "Current pose: %f %f %f %f %f %f %f",
              current_pose.position.x, current_pose.position.y,
              current_pose.position.z, current_pose.orientation.x,
              current_pose.orientation.y, current_pose.orientation.z,
              current_pose.orientation.w);




  std::vector<geometry_msgs::msg::Pose> waypoints;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double target_z = -2.80;
  double z_resolution = 0.01, z_goal_threshold = 0.01;
   while (abs(desired_pose.position.z - target_z) > z_goal_threshold) {
    desired_pose.position.z += z_resolution *
                               (target_z - desired_pose.position.z) /
                               abs(desired_pose.position.z - target_z);
    waypoints.push_back(desired_pose);
  }
  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

  move_group.execute(trajectory);
  current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(LOGGER, "Current pose: %f %f %f %f %f %f %f",
              current_pose.position.x, current_pose.position.y,
              current_pose.position.z, current_pose.orientation.x,
              current_pose.orientation.y, current_pose.orientation.z,
              current_pose.orientation.w);
  // end step
  */
  while(true){
    sleep(1);
  }
  rclcpp::shutdown();
  return 0;
}