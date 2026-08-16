cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

# -fcommon: the LKH C code declares its globals in LKH.h without extern and
# relies on common-symbol merging, which GCC >= 10 disabled by default
add_compile_options(-O3 -Wall -fcommon)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROS 2 service generation (replaces add_service_files/generate_messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveTSP.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    SYSTEM
    include
)

ament_export_include_directories(include)

file(GLOB SRCS src/*.c)

# The LKH C code and lkh_interface.cpp are ROS-free and build as-is.
# (target named *_lib because rosidl_generate_interfaces already claims
# the ${PROJECT_NAME} target name)
add_library(lkh_tsp_solver_lib SHARED
  ${SRCS}
  src/lkh_interface.cpp
)
target_link_libraries(lkh_tsp_solver_lib m)

# ------------------------------------------------------------------------
# TODO(port): tsp_node.cpp (src2/) is the ROS 1 service server wrapping
# solveTSPLKH(); re-enable after porting it to rclcpp.
#
# add_executable(tsp_node src2/tsp_node.cpp)
# ament_target_dependencies(tsp_node rclcpp std_msgs)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
# target_link_libraries(tsp_node "${cpp_typesupport_target}" lkh_tsp_solver_lib m)
# install(TARGETS tsp_node DESTINATION lib/${PROJECT_NAME})
# ------------------------------------------------------------------------

install(TARGETS lkh_tsp_solver_lib
        DESTINATION lib
)

install(DIRECTORY include/lkh_tsp_solver/
  DESTINATION include/lkh_tsp_solver
)

ament_export_libraries(lkh_tsp_solver_lib)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
