cmake_minimum_required(VERSION 3.8)
project(tf2_transforms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(my_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})

link_directories(/usr/local/lib)  

add_executable(publish_map_ned src/publish_map_ned.cpp)
add_executable(publish_odom_to_base_link src/publish_odom_to_base_link.cpp)
add_executable(publish_camera_optical src/publish_camera_optical.cpp)
add_executable(publish_aruco_marker src/publish_aruco_marker.cpp)



ament_target_dependencies(publish_map_ned
  ament_cmake
  geometry_msgs 
  px4_msgs 
  rclcpp 

  tf2_ros
)


ament_target_dependencies(publish_odom_to_base_link
  ament_cmake
  geometry_msgs 
  px4_msgs 
  rclcpp 
  tf2_ros
)


ament_target_dependencies(publish_camera_optical
  ament_cmake
  geometry_msgs 
  px4_msgs 
  rclcpp 
  tf2_ros
)

ament_target_dependencies(publish_aruco_marker
  ament_cmake
  geometry_msgs 
  px4_msgs 
  rclcpp 
  tf2_ros
  OpenCV
  cv_bridge
  my_msgs
  sensor_msgs
)


target_include_directories(publish_map_ned PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


target_include_directories(publish_odom_to_base_link PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(publish_camera_optical PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  target_include_directories(publish_aruco_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(publish_map_ned  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(publish_odom_to_base_link  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(publish_camera_optical PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(publish_aruco_marker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS publish_map_ned
  publish_odom_to_base_link
  publish_camera_optical
  publish_aruco_marker
  DESTINATION lib/${PROJECT_NAME})

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
