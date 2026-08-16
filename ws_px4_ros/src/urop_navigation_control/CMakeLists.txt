cmake_minimum_required(VERSION 3.8)
project(urop_navigation_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ego_planner REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(X11 REQUIRED)

add_executable(keyboard_flight_mode
  src/keyboard_flight_mode.cpp
)

# add_executable(autonomous_navigation_mode
#   src/autonomous_navigation_mode.cpp
# )

add_executable(lifecycle_navigation_node
  src/lifecycle_navigation_node.cpp
)

add_executable(slam_ekf_node
  src/slam_ekf2_odometry.cpp
)

add_executable(auto_nav_mode_executor
  src/autonomous_navigation_mode.cpp
  src/ascend_mode.cpp
  src/auto_nav_mode_executor.cpp

)

target_link_libraries(keyboard_flight_mode
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
  ${X11_LIBRARIES}
)

target_include_directories(keyboard_flight_mode PRIVATE
  ${X11_INCLUDE_DIR}
)

# target_include_directories(autonomous_navigation_mode PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(slam_ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(auto_nav_mode_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


ament_target_dependencies(keyboard_flight_mode
  rclcpp
)

# ament_target_dependencies(autonomous_navigation_mode
#   ament_cmake
#   geometry_msgs 
#   px4_msgs 
#   rclcpp 
#   rclcpp_lifecycle
#   lifecycle_msgs
#   quadrotor_msgs
#   sensor_msgs 
#   std_msgs  
#   px4_ros2_cpp
#   tf2_ros
#   tf2_geometry_msgs
#   ego_planner
#   octomap_msgs
# )

ament_target_dependencies(auto_nav_mode_executor
  ament_cmake
  geometry_msgs 
  px4_msgs 
  rclcpp 
  rclcpp_lifecycle
  lifecycle_msgs
  quadrotor_msgs
  sensor_msgs 
  std_msgs  
  px4_ros2_cpp
  tf2_ros
  tf2_geometry_msgs
  ego_planner
  octomap_msgs
)

ament_target_dependencies(slam_ekf_node
  ament_cmake
  geometry_msgs 
  px4_msgs 
  rclcpp 
  rclcpp_lifecycle
  lifecycle_msgs
  quadrotor_msgs
  sensor_msgs 
  std_msgs  
  px4_ros2_cpp
  tf2_ros
  tf2_geometry_msgs
  ego_planner
  octomap_msgs
  Eigen3
)

ament_target_dependencies(lifecycle_navigation_node
  ament_cmake
  geometry_msgs 
  px4_msgs 
  rclcpp 
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs 
  std_msgs  
  px4_ros2_cpp
  tf2_ros
  tf2_geometry_msgs
  ego_planner

)

install(TARGETS
  keyboard_flight_mode
  # autonomous_navigation_mode
  lifecycle_navigation_node
  slam_ekf_node
  auto_nav_mode_executor
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
