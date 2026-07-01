# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.11)
project(plc_controller)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(tecnobody_msgs REQUIRED)

generate_parameter_library(plc_controller_parameters
  src/plc_controller_parameters.yaml
)

add_library(plc_controller
  SHARED
  src/plc_controller.cpp
)
target_include_directories(plc_controller PRIVATE include)
target_link_libraries(plc_controller PUBLIC plc_controller_parameters)
ament_target_dependencies(plc_controller PUBLIC
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  tecnobody_msgs
)
pluginlib_export_plugin_description_file(controller_interface plc_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  plc_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  plc_controller
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    plc_controller_parameter_utils_test
    test/unit/test_parameter_utils.cpp
  )
  target_include_directories(plc_controller_parameter_utils_test PRIVATE include)
  target_link_libraries(plc_controller_parameter_utils_test
    plc_controller
  )

  ament_target_dependencies(
    plc_controller_parameter_utils_test
    controller_interface
  )

  ament_add_gtest(
    plc_controller_plugin_test
    test/integration/test_plugin_load.cpp
  )

  target_include_directories(plc_controller_plugin_test PRIVATE include)
  target_link_libraries(plc_controller_parameter_utils_test
    plc_controller
  )

  ament_target_dependencies(
    plc_controller_plugin_test
    controller_interface
  )


endif()
ament_package()
