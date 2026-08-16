cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

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
find_package(geometry_msgs REQUIRED)
find_package(plan_env_racer REQUIRED)
find_package(path_searching_racer REQUIRED)
find_package(active_perception REQUIRED)
find_package(lkh_tsp_solver REQUIRED)
find_package(lkh_mtsp_solver REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# ROS 2 message generation (replaces add_message_files/generate_messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneState.msg"
  "msg/PairOpt.msg"
  "msg/PairOptResponse.msg"
  "msg/HGrid.msg"
  "msg/GridTour.msg"
  DEPENDENCIES geometry_msgs
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

include_directories(
    SYSTEM
    include
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

ament_export_include_directories(include)

# ------------------------------------------------------------------------
# TODO(port): exploration_node and ground_node are NOT built yet.
#
# They depend on the deleted local-planner packages (plan_manage /
# FastPlannerManager, bspline, bspline_opt, traj_utils) and on
# quadrotor_msgs from the deleted uav_simulator. The plan is to rewrite
# the exploration FSM against ego-planner (publish next viewpoint as a
# goal) instead of porting RACER's local planner. Re-enable a target here
# once that rewrite exists.
#
# add_executable(exploration_node
#   src/exploration_node.cpp
#   src/fast_exploration_fsm.cpp
#   src/fast_exploration_manager.cpp
# )
# ament_target_dependencies(exploration_node rclcpp std_msgs geometry_msgs
#     plan_env_racer path_searching_racer active_perception
#     lkh_tsp_solver lkh_mtsp_solver cv_bridge Eigen3 PCL)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(exploration_node "${cpp_typesupport_target}" ${PCL_LIBRARIES})
# install(TARGETS exploration_node DESTINATION lib/${PROJECT_NAME})
# ------------------------------------------------------------------------

install(DIRECTORY include/exploration_manager/
  DESTINATION include/exploration_manager
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
