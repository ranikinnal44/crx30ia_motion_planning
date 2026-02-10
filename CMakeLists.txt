cmake_minimum_required(VERSION 3.8)
project(crx30ia_motion_planning)

# ------------------ Compiler options ------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------ Dependencies ------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# ------------------ Common settings ------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# ------------------ Planner / Core Nodes ------------------

add_executable(move_to_pose src/move_to_pose.cpp)
ament_target_dependencies(move_to_pose ${COMMON_DEPS})

add_executable(setup_scene src/setup_scene.cpp)
ament_target_dependencies(setup_scene
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
target_link_libraries(setup_scene
  moveit_ros_planning_interface::moveit_planning_scene_interface
)

add_executable(grasp src/grasp.cpp)
ament_target_dependencies(grasp
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(planner src/planner.cpp)
ament_target_dependencies(planner
  rclcpp
  geometry_msgs
)

# ------------------ Test / Utility Nodes ------------------

add_executable(publish_goal test/publish_goal.cpp)
ament_target_dependencies(publish_goal rclcpp geometry_msgs)

add_executable(trace_waypoints test/trace_waypoints.cpp)
ament_target_dependencies(trace_waypoints
  rclcpp
  moveit_ros_planning_interface
)

add_executable(publish_waypoints test/publish_waypoints.cpp)
ament_target_dependencies(publish_waypoints rclcpp geometry_msgs)

# ------------------ Install Targets ------------------

set(EXECUTABLES
  move_to_pose
  setup_scene
  grasp
  planner
  publish_goal
  trace_waypoints
  publish_waypoints
)

install(TARGETS ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------ Install Launch Files ------------------

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------ Testing ------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
