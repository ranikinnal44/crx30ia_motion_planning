#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

class ArmGoalPublisher : public rclcpp::Node
{
public:
  ArmGoalPublisher()
  : Node("arm_goal_publisher")
  {

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/arm_goal_pose", 10);

    timer_ = create_wall_timer(
        100ms,
        std::bind(&ArmGoalPublisher::checkAndPublish, this));

    RCLCPP_INFO(get_logger(), "ArmGoalPublisher started. Waiting for subscriber...");
  }

private:
  void checkAndPublish()
  {
    if (pub_->get_subscription_count() > 0)
    {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Subscriber detected! Publishing goal...");
      publishGoal();
    }
  }

  void publishGoal()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = this->now();

    msg.pose.position.x = 0.50;
    msg.pose.position.y = -0.20;
    msg.pose.position.z = 0.50;
    msg.pose.orientation.w = 1.0;

    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published goal. Shutting down node.");
    
    rclcpp::shutdown();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmGoalPublisher>();
  rclcpp::spin(node);
  return 0;
}
