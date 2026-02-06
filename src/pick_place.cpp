#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("arm_pick_place_node", node_options);

  // Multi-threaded executor is required for MoveGroup to update its state
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Name of the planning group defined in your MoveIt Config
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 1. Approach Red Box (Positioned at 0.8, 0.2, 0.85 in world)
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0; 
  target_pose.position.x = 0.8;
  target_pose.position.y = 0.2;
  target_pose.position.z = 1.1; // 25cm above the box
  
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 2. Lower Tool to the Box
  target_pose.position.z = 0.86; // Just touching the box surface
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 3. "Assume Grasp" - Attach box_1 to tool0
  RCLCPP_INFO(node->get_logger(), "Attaching box_1 to tool0...");
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "tool0"; // Your robot's end link
  attached_object.object.id = "box_1";
  attached_object.object.operation = attached_object.object.ADD;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);

  // 4. Lift the Box
  target_pose.position.z = 1.1;
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 5. Move to atop Blue Box (Box_2 at 1.0, -0.2, 0.85)
  target_pose.position.x = 1.0;
  target_pose.position.y = -0.2;
  target_pose.position.z = 1.05; // 0.85 (blue box) + 0.1 (red box) + safety
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 6. Lower to Place
  target_pose.position.z = 0.95; 
  move_group.setPoseTarget(target_pose);
  move_group.move();

  // 7. "Release" - Detach box_1
  RCLCPP_INFO(node->get_logger(), "Releasing box_1...");
  attached_object.object.operation = attached_object.object.REMOVE;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);

  // 8. Retreat Arm
  target_pose.position.z = 1.2;
  move_group.setPoseTarget(target_pose);
  move_group.move();

  rclcpp::shutdown();
  return 0;
}
