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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ------------------ Common settings ------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# ------------------ Planner / Core Nodes ------------------

add_executable(setup_scene src/pedestral_setup_scene.cpp)
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

add_executable(read_current_pose src/read_current_pose.cpp)
ament_target_dependencies(read_current_pose
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(camera_viewer src/camera_viewer.cpp)
ament_target_dependencies(camera_viewer
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(camera_viewer ${OpenCV_LIBS})

# ------------------ Install Targets ------------------

set(EXECUTABLES
  setup_scene
  read_current_pose
  camera_viewer
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
