cmake_minimum_required(VERSION 3.8)
project(cpp_timesyncserver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_compile_options(-DROS_DISTRO=1)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
  add_compile_options(-DROS_DISTRO=2)
else()# eloquent
  add_compile_options(-DROS_DISTRO=0)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_interfaces REQUIRED)

## Build
add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp vehicle_interfaces)

add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp vehicle_interfaces)

add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp vehicle_interfaces)

## Install
install(TARGETS
  server
  pub
  sub
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
