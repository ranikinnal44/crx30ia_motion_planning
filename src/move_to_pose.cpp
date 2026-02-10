#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

class MoveToPoseNode : public rclcpp::Node
{
public:
  explicit MoveToPoseNode(const rclcpp::NodeOptions& options)
  : Node("move_to_pose", options)
  {
    callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      10,
      std::bind(&MoveToPoseNode::goalCallback, this, std::placeholders::_1),
      sub_options);

    feedback_pub_ =
      create_publisher<std_msgs::msg::Bool>("/goal_execution_done", 10);

    RCLCPP_INFO(get_logger(),
      "MoveToPoseNode constructed. Waiting for MoveGroup initialization...");
  }

  void initMoveGroup()
  {
    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface::Options options(
      "crx30ia_arm",
      "robot_description",
      this->get_namespace());

    move_group_ =
      std::make_shared<MoveGroupInterface>(shared_from_this(), options);

    move_group_->setPoseReferenceFrame("world");
    move_group_->setEndEffectorLink("EOAT_link");

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    RCLCPP_INFO(get_logger(),
      "MoveGroup initialized for group: %s",
      move_group_->getName().c_str());
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      RCLCPP_WARN(get_logger(),
        "Already executing a goal, ignoring new request.");
      return;
    }

    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();

    move_group_->setPoseTarget(msg->pose);

    RCLCPP_INFO(get_logger(),
      "Planning to pose [%.2f %.2f %.2f]",
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group_->execute(plan);
      RCLCPP_INFO(get_logger(), "Execution complete.");

      std_msgs::msg::Bool feedback;
      feedback.data = true;
      feedback_pub_->publish(feedback);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Planning failed.");

      std_msgs::msg::Bool feedback;
      feedback.data = false;
      feedback_pub_->publish(feedback);
    }
  }

  // ---- Members ----
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feedback_pub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::mutex mutex_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<MoveToPoseNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  node->initMoveGroup();
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
