#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("setup_scene_node", 
    rclcpp::NodeOptions().use_intra_process_comms(true));

  // The PlanningSceneInterface is the bridge to MoveIt
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // We will create a vector of collision objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  // --- 1. Define the Table ---
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "table";
  
  shape_msgs::msg::SolidPrimitive table_shape;
  table_shape.type = table_shape.BOX;
  table_shape.dimensions = {1.0, 1.5, 0.8}; // X, Y, Z

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 1.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.4; // Center of 0.8m height
  
  table.primitives.push_back(table_shape);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;
  collision_objects.push_back(table);

  // --- 2. Define Box 1 (Red) ---
  moveit_msgs::msg::CollisionObject box1;
  box1.header.frame_id = "world";
  box1.id = "box_1";
  
  shape_msgs::msg::SolidPrimitive box_shape;
  box_shape.type = box_shape.BOX;
  box_shape.dimensions = {0.1, 0.1, 0.1};

  geometry_msgs::msg::Pose box1_pose;
  box1_pose.position.x = 0.8;
  box1_pose.position.y = 0.2;
  box1_pose.position.z = 0.85; // 0.8 (table) + 0.05 (half box height)
  
  box1.primitives.push_back(box_shape);
  box1.primitive_poses.push_back(box1_pose);
  box1.operation = box1.ADD;
  collision_objects.push_back(box1);

  // --- 3. Define Box 2 (Blue) ---
  moveit_msgs::msg::CollisionObject box2 = box1; // Copy shape
  box2.id = "box_2";
  box2.primitive_poses[0].position.x = 1.0;
  box2.primitive_poses[0].position.y = -0.2;
  
  collision_objects.push_back(box2);

  // Add objects to the planning scene
  RCLCPP_INFO(node->get_logger(), "Adding objects to the planning scene...");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Keep the node alive for a moment to ensure the message is sent
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  rclcpp::shutdown();
  return 0;
}
