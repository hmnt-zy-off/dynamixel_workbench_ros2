cmake_minimum_required(VERSION 3.8)
project(dynamixel_workbench_operators)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(LIB_NAME "dynamixel_workbench_operators")

find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# find_package(dynamixel_workbench_msgs REQUIRED)
# find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(yaml-cpp REQUIRED)

# Build
add_executable(joint_operator
  src/joint_operator.cpp
)

add_executable(wheel_operator
  src/wheel_operator.cpp
)

# Link non-ROS dependencies
target_link_libraries(joint_operator
  yaml-cpp
)


# Link ROS 2 and Dynamixel dependencies
ament_target_dependencies(joint_operator
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

ament_target_dependencies(wheel_operator
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# Include directories
target_include_directories(joint_operator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_include_directories(wheel_operator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Install
install(TARGETS
  joint_operator
  wheel_operator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

ament_package()