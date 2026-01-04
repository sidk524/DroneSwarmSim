cmake_minimum_required(VERSION 3.8)
project(baseline_autonomous_exploration_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

add_executable(frontierFinder src/frontierFinder.cpp)
ament_target_dependencies(frontierFinder
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  octomap_msgs
  octomap
)
target_link_libraries(frontierFinder ${OCTOMAP_LIBRARIES})
target_include_directories(frontierFinder PUBLIC ${OCTOMAP_INCLUDE_DIRS})

install(TARGETS
  frontierFinder
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
