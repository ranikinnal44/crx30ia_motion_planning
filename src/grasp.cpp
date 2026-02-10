#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <std_msgs/msg/empty.hpp>

class AttachBoxNode : public rclcpp::Node
{
public:
  AttachBoxNode() : Node("attach_box_node")
  {
    attach_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/attach_box_trigger",
      10,
      std::bind(&AttachBoxNode::onAttachTriggered, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "AttachBoxNode ready. Waiting for attach trigger...");
  }

private:
  void onAttachTriggered(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    RCLCPP_INFO(get_logger(), "Attach trigger received");

    attachBox("box_1");
  }

  void attachBox(const std::string& box_id)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = "EOAT_link";
    aco.object.id = box_id;
    aco.object.operation = aco.object.ADD;

    // Allow collision between EOAT and box
    aco.touch_links = {"EOAT_link", "J6_link"};

    planning_scene_interface_.applyAttachedCollisionObject(aco);

    RCLCPP_INFO(
      get_logger(),
      "Object '%s' attached to EOAT_link",
      box_id.c_str());
  }

  // ---- Members ----
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr attach_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AttachBoxNode>());
  rclcpp::shutdown();
  return 0;
}
