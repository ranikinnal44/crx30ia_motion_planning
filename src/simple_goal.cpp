#include <memory>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/planning_scene/planning_scene.h>

class ArmGoalNode : public rclcpp::Node
{
public:
  explicit ArmGoalNode(const rclcpp::NodeOptions& options)
  : Node("simple_goal", options)
  {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "arm_goal_pose",
        10,
        std::bind(&ArmGoalNode::goalCallback, this, std::placeholders::_1),
        sub_options);

    RCLCPP_INFO(get_logger(), "ArmGoalNode constructed. Waiting for MoveGroup...");
  }

  void initMoveGroup()
  {
    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface::Options moveit_options(
        "crx30ia_arm",
        "robot_description",
        this->get_namespace() 
    );

    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), moveit_options);

    RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized for group: %s", 
                move_group_->getName().c_str());
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        RCLCPP_WARN(get_logger(), "Already processing a goal, ignoring new request.");
        return;
    }

    if (!move_group_)
    {
      RCLCPP_WARN(get_logger(), "MoveGroup not ready yet");
      return;
    }
    auto current_state = move_group_->getCurrentState(10.0); 
    
    if (!current_state)
    {
      RCLCPP_ERROR(get_logger(), "Failed to fetch current robot state. ");
      return;
    }

    geometry_msgs::msg::Pose target_pose = msg->pose;
    move_group_->setPoseTarget(target_pose);

    RCLCPP_INFO(get_logger(), "Planning to pose target...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(get_logger(), "Planning successful, executing...");
      move_group_->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Planning failed!");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::mutex mutex_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<ArmGoalNode>(options);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->initMoveGroup();

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
