cmake_minimum_required(VERSION 3.8)
project(crx30ia_motion_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(simple_goal src/simple_goal.cpp)

add_executable(trace_waypoints src/trace_waypoints.cpp)

add_executable(publish_waypoints src/publish_waypoints.cpp)

add_executable(setup_scene src/setup_scene.cpp)

add_executable(publish_goal src/publish_goal.cpp)

target_include_directories(simple_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(simple_goal PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  simple_goal
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Install executable
install(TARGETS simple_goal
  DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(
  trace_waypoints
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS trace_waypoints
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  publish_waypoints
  rclcpp
  geometry_msgs
)

install(TARGETS publish_waypoints
  DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(
  publish_goal
  rclcpp
  geometry_msgs
)

install(TARGETS publish_goal
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(setup_scene
  moveit_ros_planning_interface::moveit_planning_scene_interface
)
ament_target_dependencies(setup_scene
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
)

install(TARGETS setup_scene
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# --------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
