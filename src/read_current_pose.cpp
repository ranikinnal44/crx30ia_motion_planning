#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class CurrentPosePublisher : public rclcpp::Node
{
public:
  explicit CurrentPosePublisher(const rclcpp::NodeOptions& options)
  : Node("current_pose_publisher", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/current_ee_pose", 10);

    timer_ = create_wall_timer(
      200ms,
      std::bind(&CurrentPosePublisher::publishPose, this));

    RCLCPP_INFO(get_logger(), "TF-based CurrentPosePublisher started.");
  }

private:
  void publishPose()
  {
    try
    {
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(
              "world",        // target frame
              "EOAT_link",    // source frame
              tf2::TimePointZero);  // latest available

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = transform.header;

      pose_msg.pose.position.x = transform.transform.translation.x;
      pose_msg.pose.position.y = transform.transform.translation.y;
      pose_msg.pose.position.z = transform.transform.translation.z;
      pose_msg.pose.orientation = transform.transform.rotation;

      publisher_->publish(pose_msg);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "TF not ready yet...");
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<CurrentPosePublisher>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
