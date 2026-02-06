#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>

using namespace std::chrono_literals;

class WaypointPublisher : public rclcpp::Node {
public:
  WaypointPublisher() : Node("waypoint_publisher") {
    // Publisher for PoseArray on the topic 'external_waypoints'
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("external_waypoints", 10);

    // Poll every 100ms to check if the subscriber is active
    timer_ = this->create_wall_timer(100ms, std::bind(&WaypointPublisher::checkAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "Waypoint Publisher started. Waiting for subscriber...");
  }

private:
  void checkAndPublish() {
    if (publisher_->get_subscription_count() > 0) {
      timer_->cancel(); // Only publish once
      publishWaypoints();
    }
  }

  void publishWaypoints() {
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = "arm_base_link";
    msg.header.stamp = this->now();

    // Define points relative to your robot's workspace
    geometry_msgs::msg::Pose p1, p2, p3;

    // Waypoint 1
    p1.position.x = 0.50; p1.position.y = 0.0; p1.position.z = 0.50;
    p1.orientation.w = 1.0;

    // Waypoint 2 (shift in X and Z)
    p2.position.x = 0.60; p2.position.y = 0.0; p2.position.z = 0.55;
    p2.orientation.w = 1.0;

    // Waypoint 3 (shift in Y)
    p3.position.x = 0.60; p3.position.y = -0.10; p3.position.z = 0.55;
    p3.orientation.w = 1.0;

    // Populate the array
    msg.poses.push_back(p1);
    msg.poses.push_back(p2);
    msg.poses.push_back(p3);

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published %zu waypoints. Shutting down.", msg.poses.size());

    rclcpp::shutdown();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointPublisher>();
  rclcpp::spin(node);
  return 0;
}
