cmake_minimum_required(VERSION 3.8)
project(dynaarm_gamepad_interface)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wno-c++20-extensions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(kinematic_utils src/kinematic_utils.cpp)
target_include_directories(kinematic_utils
    PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}  # Add Eigen3 include directories
)
ament_target_dependencies(kinematic_utils
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  pinocchio
  Eigen3)

add_executable(gamepad_node
  src/gamepad_node.cpp
  src/gamepad_handler.cpp
  src/gamepad_receiver.cpp
  src/controller_helper.cpp
  src/kinematic_utils.cpp)
target_include_directories(gamepad_node
  PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}  # Add Eigen3 include directories
)
target_link_libraries(gamepad_node
  ${rclcpp_LIBRARIES}
  ${rclcpp_components_LIBRARIES})
ament_target_dependencies(gamepad_node
  rclcpp
  sensor_msgs
  controller_manager_msgs
  trajectory_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  pinocchio
  Eigen3
)

install(
  TARGETS gamepad_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
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
