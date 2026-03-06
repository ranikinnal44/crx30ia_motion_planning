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
    rclcpp::sleep_for(std::chrono::seconds(2));

    buildBoxStack();
    buildConveyor();

    planning_scene_interface_.applyCollisionObjects(collision_objects_);
    applyColors();

    RCLCPP_INFO(get_logger(),
      "PlanningScene: %zu collision objects added.",
      collision_objects_.size());
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::msg::CollisionObject>     collision_objects_;
  std::vector<moveit_msgs::msg::ObjectColor>         object_colors_;

  void buildBoxStack()
  {
    // Box stack position - moved farther from robot for pick flexibility
    // X = 1.10m gives robot room to approach
    const double STACK_X = 1.10;

    // Reduced stack: 3 columns x 2 rows (top row at Z=0.675 is reachable)
    // Column positions (Y): center and adjacent
    const std::array<double, 3> col_y = { 0.45, 0.0, -0.45 };
    // Row positions (Z): only 2 rows - top row is reachable
    const std::array<double, 2> row_z = { 0.225, 0.675 };

    for (size_t r = 0; r < row_z.size(); ++r)
    {
      for (size_t c = 0; c < col_y.size(); ++c)
      {
        const std::string id =
          "box_c" + std::to_string(c) + "_r" + std::to_string(r);

        const bool is_red = ((c + r) % 2 == 0);
        const auto color  = is_red
          ? makeColor(0.8, 0.2, 0.2, 1.0)
          : makeColor(0.2, 0.2, 0.8, 1.0);

        addBoxObject(
          id,
          {0.45, 0.45, 0.45},
          makePose(STACK_X, col_y[c], row_z[r]),
          color
        );
      }
    }
  }

  void buildConveyor()
  {
    // Belt surface
    addBoxObject(
      "conveyor_belt",
      {1.5, 0.6, 0.05},
      makePose(-0.195, -1.2, 0.725),
      makeColor(0.1, 0.1, 0.1, 1.0)
    );

    // Support frame
    addBoxObject(
      "conveyor_frame",
      {1.4, 0.5, 0.70},
      makePose(-0.195, -1.2, 0.350),
      makeColor(0.5, 0.5, 0.5, 1.0)
    );

    // Front roller  world X = -0.195 + 0.75 = 0.555
    addCylinderObject(
      "conveyor_roller_front",
      0.04, 0.6,
      makePoseRPY(0.555, -1.2, 0.725,  M_PI_2, 0.0, 0.0),
      makeColor(0.3, 0.3, 0.3, 1.0)
    );

    // Rear roller   world X = -0.195 - 0.75 = -0.945
    addCylinderObject(
      "conveyor_roller_rear",
      0.04, 0.6,
      makePoseRPY(-0.945, -1.2, 0.725,  M_PI_2, 0.0, 0.0),
      makeColor(0.3, 0.3, 0.3, 1.0)
    );
  }

  // ═══════════════════════════════════════════════════════════════════════
  //  Helpers
  // ═══════════════════════════════════════════════════════════════════════

  geometry_msgs::msg::Pose makePose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = 1.0;
    return p;
  }

  geometry_msgs::msg::Pose makePoseRPY(
    double x, double y, double z,
    double roll, double pitch, double yaw)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    const double cr = std::cos(roll  * 0.5), sr = std::sin(roll  * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw   * 0.5), sy = std::sin(yaw   * 0.5);
    p.orientation.w = cr*cp*cy + sr*sp*sy;
    p.orientation.x = sr*cp*cy - cr*sp*sy;
    p.orientation.y = cr*sp*cy + sr*cp*sy;
    p.orientation.z = cr*cp*sy - sr*sp*cy;
    return p;
  }

  moveit_msgs::msg::ObjectColor makeColor(double r, double g, double b, double a)
  {
    moveit_msgs::msg::ObjectColor c;
    c.color.r = static_cast<float>(r);
    c.color.g = static_cast<float>(g);
    c.color.b = static_cast<float>(b);
    c.color.a = static_cast<float>(a);
    return c;
  }

  void addBoxObject(
    const std::string                    &id,
    const std::array<double, 3>          &size,
    const geometry_msgs::msg::Pose       &pose,
    const moveit_msgs::msg::ObjectColor  &color)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = id;
    obj.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type       = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {size[0], size[1], size[2]};
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects_.push_back(obj);
    moveit_msgs::msg::ObjectColor c = color;
    c.id = id;
    object_colors_.push_back(c);
  }

  void addCylinderObject(
    const std::string                    &id,
    double                                radius,
    double                                length,
    const geometry_msgs::msg::Pose       &pose,
    const moveit_msgs::msg::ObjectColor  &color)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.id              = id;
    obj.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive prim;
    prim.type       = shape_msgs::msg::SolidPrimitive::CYLINDER;
    prim.dimensions = {length, radius};  // [0]=height along Z, [1]=radius
    obj.primitives.push_back(prim);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects_.push_back(obj);
    moveit_msgs::msg::ObjectColor c = color;
    c.id = id;
    object_colors_.push_back(c);
  }

  void applyColors()
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff        = true;
    scene.object_colors  = object_colors_;
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
