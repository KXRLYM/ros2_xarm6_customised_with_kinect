cmake_minimum_required(VERSION 3.8)
project(xarm_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(xarm_api REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(action_msgs REQUIRED)

add_executable(xarm_move_client src/xarm_move_client.cpp)
ament_target_dependencies(xarm_move_client action_msgs xarm_msgs yolov8_msgs geometry_msgs rclcpp moveit_core moveit_msgs moveit_ros_planning_interface moveit_ros_planning moveit_kinematics moveit_visual_tools xarm_api xarm_msgs tf2_ros tf2_geometry_msgs)

target_link_libraries(xarm_move_client
  ${rclcpp_LIBRARIES}
  ${tf2_ros_LIBRARIES}  # Add this line
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executable
install(TARGETS
  xarm_move_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
