cmake_minimum_required(VERSION 3.5)
project(trace_blueye_ros_connector)

# Use C++14
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Wpedantic -fPIC)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trace_executive REQUIRED)
find_package(blueye_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time program_options serialization regex)

# Include dirs
include_directories(
  include
  ${trace_executive_INCLUDE_DIRS}
)

# Build shared library (TRACE plugin)
add_library(${PROJECT_NAME} SHARED
  src/connector.cc
  src/specific_client.cc
)

# Rename output so TRACE can discover plugin
set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME "trace_blueye_ros_connector"
)

# Dependencies for library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  trace_executive
  blueye_interfaces
  std_msgs
)

# Install shared library
install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install XML config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export library so TRACE can load it dynamically
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp trace_executive blueye_interfaces std_msgs)

ament_package()