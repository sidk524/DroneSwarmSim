cmake_minimum_required(VERSION 3.8)
project(active_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env_racer REQUIRED)
find_package(path_searching_racer REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

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

include_directories(
    SYSTEM
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

ament_export_include_directories(include)

# NOTE: traj_visibility.cpp removed - it was the only file depending on the
# deleted `bspline` package (NonUniformBspline), and it is unused by the
# frontier-exploration pipeline.
add_library(active_perception SHARED
    src/heading_planner.cpp
    src/frontier_finder.cpp
    src/graph_node.cpp
    src/perception_utils.cpp
    src/uniform_grid.cpp
    src/hgrid.cpp
)

ament_target_dependencies(active_perception rclcpp std_msgs visualization_msgs
    plan_env_racer path_searching_racer cv_bridge Eigen3 PCL pcl_conversions)

target_link_libraries(active_perception
    ${PCL_LIBRARIES}
)

install(TARGETS active_perception
        DESTINATION lib
)

install(DIRECTORY include/active_perception/
  DESTINATION include/active_perception
)

ament_export_libraries(active_perception)
ament_export_dependencies(plan_env_racer path_searching_racer)

ament_package()
