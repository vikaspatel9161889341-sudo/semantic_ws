cmake_minimum_required(VERSION 3.8)
project(nav_failsafe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(cmd_vel_to_px4_bridge_node src/cmd_vel_to_px4_bridge.cpp)
target_compile_features(cmd_vel_to_px4_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(cmd_vel_to_px4_bridge_node rclcpp geometry_msgs px4_msgs)

add_library(battery_emergency_condition_plugin SHARED src/BatteryEmergencyCondition.cpp)
target_compile_features(battery_emergency_condition_plugin PUBLIC cxx_std_17)
ament_target_dependencies(battery_emergency_condition_plugin rclcpp px4_msgs behaviortree_cpp_v3)

# Folders ko install directory mein bhejeb ke naye rules
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS cmd_vel_to_px4_bridge_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS battery_emergency_condition_plugin DESTINATION lib/${PROJECT_NAME})

ament_package()
