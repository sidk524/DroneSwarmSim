cmake_minimum_required(VERSION 3.8)
project(precision_landing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_executable(precision_landing_mode_executor src/precision_landing_mode_executor.cpp)
add_executable(initial_fly_up src/initial_fly_up_mode.cpp)
add_executable(locate_marker src/locate_marker.cpp)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

ament_target_dependencies(precision_landing_mode_executor ament_cmake)
ament_target_dependencies(precision_landing_mode_executor geometry_msgs)
ament_target_dependencies(precision_landing_mode_executor px4_msgs)
ament_target_dependencies(precision_landing_mode_executor rclcpp)
ament_target_dependencies(precision_landing_mode_executor sensor_msgs)
ament_target_dependencies(precision_landing_mode_executor std_msgs)
ament_target_dependencies(precision_landing_mode_executor px4_ros2_cpp)

ament_target_dependencies(initial_fly_up ament_cmake)
ament_target_dependencies(initial_fly_up geometry_msgs)
ament_target_dependencies(initial_fly_up px4_msgs)
ament_target_dependencies(initial_fly_up rclcpp)
ament_target_dependencies(initial_fly_up sensor_msgs)
ament_target_dependencies(initial_fly_up std_msgs)
ament_target_dependencies(initial_fly_up px4_ros2_cpp)

ament_target_dependencies(locate_marker ament_cmake)
ament_target_dependencies(locate_marker geometry_msgs)
ament_target_dependencies(locate_marker px4_msgs)
ament_target_dependencies(locate_marker rclcpp)
ament_target_dependencies(locate_marker sensor_msgs)
ament_target_dependencies(locate_marker std_msgs)
ament_target_dependencies(locate_marker px4_ros2_cpp)
ament_target_dependencies(locate_marker cv_bridge)
ament_target_dependencies(locate_marker OpenCV)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

target_include_directories(precision_landing_mode_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

  target_include_directories(initial_fly_up PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
    
target_include_directories(locate_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(precision_landing_mode_executor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(initial_fly_up PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(locate_marker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS precision_landing_mode_executor
  initial_fly_up
  locate_marker
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
