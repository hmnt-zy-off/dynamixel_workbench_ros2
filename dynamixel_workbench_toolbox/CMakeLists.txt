cmake_minimum_required(VERSION 3.8)
project(dynamixel_workbench_toolbox)

# Make c++ 17 as the standard since both humble and jazzy supports
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)


#build
set(LIB_NAME "dynamixel_workbench_toolbox")

add_library(${LIB_NAME} SHARED
  src/${PROJECT_NAME}/dynamixel_item.cpp
  src/${PROJECT_NAME}/dynamixel_tool.cpp
  src/${PROJECT_NAME}/dynamixel_driver.cpp
  src/${PROJECT_NAME}/dynamixel_workbench.cpp
  )

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${dynamixel_sdk_INCLUDE_DIRS})

target_link_libraries(${LIB_NAME} PUBLIC
  ${dynamixel_sdk_LIBRARIES}
)

#install
install(TARGETS ${LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(dynamixel_sdk)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
