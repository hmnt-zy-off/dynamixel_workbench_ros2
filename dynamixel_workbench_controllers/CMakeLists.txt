cmake_minimum_required(VERSION 3.8)
project(dynamixel_workbench_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(LIB_NAME "dynamixel_workbench_controllers")

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build
add_executable(find_dynamixel src/find_dynamixel.cpp)
add_executable(dynamixel_workbench_controllers
  src/dynamixel_workbench_controllers.cpp
  src/trajectory_generator.cpp
)

# Link non-ROS dependencies
target_link_libraries(dynamixel_workbench_controllers
  yaml-cpp
  # Eigen3::Eigen
)

# Link ROS 2 and Dynamixel dependencies
ament_target_dependencies(find_dynamixel
  rclcpp
  std_msgs
  dynamixel_workbench_toolbox
)
ament_target_dependencies(dynamixel_workbench_controllers
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)

# Include directories
target_include_directories(find_dynamixel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(dynamixel_workbench_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Install
install(TARGETS
  find_dynamixel
  dynamixel_workbench_controllers
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
  rclcppsensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)

ament_package()