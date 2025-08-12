cmake_minimum_required(VERSION 3.8)
project(tfluna)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tca9548a REQUIRED)

include_directories(include)

add_library(tfluna_driver SHARED
  src/TFLI2C.cpp
  src/tfluna_device.cpp
)

# Use ament_target_dependencies for external ROS packages
ament_target_dependencies(tfluna_driver
  "rclcpp"
  "pluginlib"
  "tca9548a"
)

pluginlib_export_plugin_description_file(tfluna_driver tfluna_plugins.xml)

install(TARGETS tfluna_driver
  EXPORT tfluna_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(tfluna_driver_targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  pluginlib
  tca9548a
  sensor_msgs
)

ament_package()