cmake_minimum_required(VERSION 3.8)
project(dynamixel_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# === Dynamixel Driver Node ===
add_executable(dynamixel_driver_node
  src/dynamixel_driver_node.cpp
  src/drive_modes.cpp
)
ament_target_dependencies(dynamixel_driver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  dynamixel_sdk
)
target_include_directories(dynamixel_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# === Joystick Mapper Node ===
add_executable(joy_mapper_node
  src/joy_mapper_node.cpp
)
ament_target_dependencies(joy_mapper_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install executables
install(TARGETS
  dynamixel_driver_node
  joy_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
