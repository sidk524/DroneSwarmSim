cmake_minimum_required(VERSION 3.8)
project(plan_env_racer)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# ROS 2 message generation (replaces add_message_files/generate_messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IdxList.msg"
  "msg/ChunkStamps.msg"
  "msg/ChunkData.msg"
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
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

ament_export_include_directories(include)

add_library(plan_env_racer_lib SHARED
    src/sdf_map.cpp
    src/obj_predictor.cpp
    src/edt_environment.cpp
    src/raycast.cpp
    src/map_ros.cpp
    src/multi_map_manager.cpp
)

ament_target_dependencies(plan_env_racer_lib rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs
    visualization_msgs cv_bridge message_filters Eigen3 PCL pcl_conversions)

# link this package's own generated messages (ChunkData etc. used by multi_map_manager)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(plan_env_racer_lib
    "${cpp_typesupport_target}"
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(obj_generator
    src/obj_generator.cpp
)
ament_target_dependencies(obj_generator rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs
    visualization_msgs Eigen3 PCL pcl_conversions)

install(TARGETS plan_env_racer_lib
        DESTINATION lib
)

install(TARGETS obj_generator
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/plan_env/
  DESTINATION include/plan_env
)

ament_export_libraries(plan_env_racer_lib)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
