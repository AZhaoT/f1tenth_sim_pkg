cmake_minimum_required(VERSION 3.5)
project(f1tenth_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/ScanRange.msg"
  "msg/PidInput.msg"
  "msg/GapDir.msg"
  "msg/OppGapDir.msg"
  "msg/Transformation.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(laserscan_subscriber src/laserscan_sub.cpp)
add_executable(auto_brake src/safety_node.cpp)
add_executable(dist_finder src/dist_finder.cpp)
add_executable(control src/control.cpp)
add_executable(gap_finder src/reactive_gap_follow.cpp)
add_executable(drive_to_gap src/drive_to_gap.cpp)
add_executable(opp_gap_finder src/opp_reactive_gap_follow.cpp)
add_executable(opp_drive_to_gap src/opp_drive_to_gap.cpp)
add_executable(scan_match_transform src/transform.cpp)



ament_target_dependencies(laserscan_subscriber rclcpp std_msgs sensor_msgs)
ament_target_dependencies(auto_brake rclcpp std_msgs sensor_msgs nav_msgs ackermann_msgs)
ament_target_dependencies(dist_finder rclcpp std_msgs sensor_msgs)
ament_target_dependencies(control rclcpp std_msgs sensor_msgs ackermann_msgs)
ament_target_dependencies(gap_finder rclcpp std_msgs sensor_msgs)
ament_target_dependencies(drive_to_gap rclcpp std_msgs ackermann_msgs)
ament_target_dependencies(opp_gap_finder rclcpp std_msgs sensor_msgs)
ament_target_dependencies(opp_drive_to_gap rclcpp std_msgs ackermann_msgs)
ament_target_dependencies(scan_match_transform rclcpp std_msgs sensor_msgs)



install(TARGETS
	laserscan_subscriber
  auto_brake
  dist_finder
  control
  gap_finder
  drive_to_gap
  opp_gap_finder
  opp_drive_to_gap
  scan_match_transform
	DESTINATION lib/${PROJECT_NAME}
)

rosidl_target_interfaces(laserscan_subscriber 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(auto_brake 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(dist_finder 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(control 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(gap_finder 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(drive_to_gap
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(opp_gap_finder 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(opp_drive_to_gap
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(scan_match_transform
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
