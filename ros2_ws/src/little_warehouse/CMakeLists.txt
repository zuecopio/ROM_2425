################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################

cmake_minimum_required(VERSION 3.8)
project(little_warehouse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(navigation_node 
  src/navigation_node.cpp)

ament_target_dependencies(navigation_node
  rclcpp
  yaml-cpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

target_link_libraries(navigation_node
  yaml-cpp
)

################################################################################
# Install
################################################################################

# Install folder (executables inside)
install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# To launch files
install(DIRECTORY launch params scenes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################

ament_export_include_directories(include)
ament_package()