#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>

class PoseSequencePlanner : public rclcpp::Node
{
public:
  PoseSequencePlanner()
  : Node("pose_sequence_planner")
  {
    goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    attach_pub_ =
      create_publisher<std_msgs::msg::Empty>("/attach_box_trigger", 10);

    feedback_sub_ =
      create_subscription<std_msgs::msg::Bool>(
        "/goal_execution_done",
        10,
        std::bind(&PoseSequencePlanner::onExecutionDone, this, std::placeholders::_1));

    buildGoals();
    publishNextGoal();

    RCLCPP_INFO(get_logger(),
      "Planner started with %zu goals", goals_.size());
  }

private:
  /* ---------------- TASK DEFINITION ---------------- */

  void buildGoals()
  {
    // 0 → pre-grasp
    goals_.push_back(makePose(0.9, 0.3, 0.9));

    // 1 → grasp pose (attach after this)
    // goals_.push_back(makePose(0.9, 0.4, 0.9));

    // 2 → lift
    goals_.push_back(makePose(0.9, 0.2, 0.9));
  }

  geometry_msgs::msg::PoseStamped makePose(double x, double y, double z)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "world";
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;

    // EOAT downward
    p.pose.orientation.x = 1.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 0.0;

    return p;
  }

  /* ---------------- EXECUTION FLOW ---------------- */

  void publishNextGoal()
  {
    if (current_index_ >= goals_.size())
    {
      RCLCPP_INFO(get_logger(), "All goals completed.");
      return;
    }

    auto& goal = goals_[current_index_];
    goal.header.stamp = now();
    goal_pub_->publish(goal);

    RCLCPP_INFO(get_logger(),
      "Published goal %zu", current_index_);
  }

  void onExecutionDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data)
    {
      RCLCPP_ERROR(get_logger(), "Motion failed. Stopping planner.");
      return;
    }

    RCLCPP_INFO(get_logger(),
      "Execution finished for goal %zu", current_index_);

    // 🔴 Attach AFTER grasp pose (index 1)
    if (current_index_ == 1)
    {
      RCLCPP_INFO(get_logger(), "Triggering attach box");
      std_msgs::msg::Empty trigger;
      attach_pub_->publish(trigger);

      // small delay to let planning scene update
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    current_index_++;
    publishNextGoal();
  }

  /* ---------------- MEMBERS ---------------- */

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr feedback_sub_;

  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  size_t current_index_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSequencePlanner>());
  rclcpp::shutdown();
  return 0;
}
