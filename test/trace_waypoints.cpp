#include <memory>
#include <thread>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp> // Use PoseArray for multiple waypoints

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class WaypointFollower : public rclcpp::Node {
public:
  explicit WaypointFollower(const rclcpp::NodeOptions& options)
  : Node("trace_waypoints", options) {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "external_waypoints", 10,
        std::bind(&WaypointFollower::waypointCallback, this, std::placeholders::_1),
        sub_options);

    RCLCPP_INFO(this->get_logger(), "Node started. Waiting for waypoints on 'external_waypoints'...");
  }

  void initMoveGroup() {
    moveit::planning_interface::MoveGroupInterface::Options moveit_options(
        "crx30ia_arm", "robot_description", this->get_namespace());

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), moveit_options);
    move_group_->setPlanningTime(10.0);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);
    
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized");
  }

private:
  void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(execution_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      RCLCPP_WARN(this->get_logger(), "Planning already in progress, dropping message.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %zu waypoints. Starting validation...", msg->poses.size());

    auto current_state = move_group_->getCurrentState(10.0);
    if (!current_state) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state. Is /joint_states active?");
      return;
    }

    planning_scene::PlanningScene planning_scene(move_group_->getRobotModel());
    const auto* jmg = current_state->getJointModelGroup("crx30ia_arm");

    std::vector<geometry_msgs::msg::Pose> waypoints = msg->poses;

    /* ---------------- Validation ---------------- */
    for (size_t i = 0; i < waypoints.size(); ++i) {
      moveit::core::RobotState test_state(*current_state);
      if (!test_state.setFromIK(jmg, waypoints[i], 0.1)) {
        RCLCPP_ERROR(this->get_logger(), "Waypoint %zu: No IK solution", i);
        return;
      }
      collision_detection::CollisionRequest req;
      collision_detection::CollisionResult res;
      planning_scene.checkCollision(req, res, test_state);
      if (res.collision) {
        RCLCPP_ERROR(this->get_logger(), "Waypoint %zu: In collision", i);
        return;
      }
    }

    /* ------------ Cartesian Planning ------------ */
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;
    const double jump_threshold = 0.0; // Standard MoveIt 2 default

    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(this->get_logger(), "Cartesian path coverage: %.2f%%", fraction * 100.0);

    if (fraction < 0.75) {
      RCLCPP_ERROR(this->get_logger(), "Insufficient Cartesian path coverage");
      return;
    }

    /* ---------------- Execution ---------------- */
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::mutex execution_mutex_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<WaypointFollower>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  node->initMoveGroup();
  
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
