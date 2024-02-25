cmake_minimum_required(VERSION 3.5)
project(robot_frame_provider)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(biped_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rot_conv REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  biped_interfaces
  Eigen3
  rclcpp
  rclcpp_components
  rot_conv
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros)

# Build
add_library(frames_broadcaster_node SHARED
  src/frames_broadcaster.cpp)
add_library(phase_publisher_node SHARED
  src/phase_publisher.cpp)

target_include_directories(frames_broadcaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(phase_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(frames_broadcaster_node
  PLUGIN "FramesBroadcaster::FramesBroadcaster"
  EXECUTABLE frames_broadcaster)
rclcpp_components_register_node(phase_publisher_node
  PLUGIN "PhasePublisher::PhasePublisher"
  EXECUTABLE phase_publisher)

ament_target_dependencies(frames_broadcaster_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(phase_publisher_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS frames_broadcaster_node phase_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Executable
add_executable(main
  src/main.cpp
)

target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(main
  frames_broadcaster_node
  phase_publisher_node
  ${ament_LIBRARIES}
)

# Install executable
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

# Package
ament_package()
