#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class SceneSetupNode : public rclcpp::Node
{
public:
  SceneSetupNode() : Node("scene_setup_node")
  {
    // Allow move_group to initialize
    rclcpp::sleep_for(std::chrono::seconds(2));

    addBoxObject(
      "conveyor",
      {1.5, 0.6, 0.05},
      makePose(0.5, -1.2, 0.725),
      makeColor(0.2, 0.2, 0.2, 1.0)
    );

    addBoxObject(
      "box_1",
      {0.6, 0.6, 0.6},
      makePose(1.0, 0.3, 0.3),
      makeColor(1.0, 0.0, 0.0, 1.0)
    );

    addBoxObject(
      "box_2",
      {0.6, 0.6, 0.6},
      makePose(1.0, -0.3, 0.3),
      makeColor(0.0, 0.0, 1.0, 1.0)
    );

    // ----------- Apply to MoveIt -----------
    planning_scene_interface_.applyCollisionObjects(collision_objects_);
    applyColors();

    RCLCPP_INFO(get_logger(), "PlanningScene objects and colors added.");
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects_;
  std::vector<moveit_msgs::msg::ObjectColor> object_colors_;

  /* ================= HELPERS ================= */

  geometry_msgs::msg::Pose makePose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 1.0;
    return p;
  }

  moveit_msgs::msg::ObjectColor makeColor(double r, double g, double b, double a)
  {
    moveit_msgs::msg::ObjectColor c;
    c.color.r = r;
    c.color.g = g;
    c.color.b = b;
    c.color.a = a;
    return c;
  }

  void addBoxObject(
    const std::string &id,
    const std::array<double, 3> &size,
    const geometry_msgs::msg::Pose &pose,
    const moveit_msgs::msg::ObjectColor &color)
  {
    // ---- Collision object ----
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {size[0], size[1], size[2]};

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    obj.operation = obj.ADD;

    collision_objects_.push_back(obj);

    // ---- Color ----
    moveit_msgs::msg::ObjectColor c = color;
    c.id = id;
    object_colors_.push_back(c);
  }

  void applyColors()
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;

    for (const auto &color : object_colors_)
    {
      scene.object_colors.push_back(color);
    }

    planning_scene_interface_.applyPlanningScene(scene);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SceneSetupNode>();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
