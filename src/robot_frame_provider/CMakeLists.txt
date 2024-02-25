cmake_minimum_required(VERSION 3.8)
project(robot_frame_provider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(biped_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rot_conv REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(base_link_broadcaster
  src/base_link_broadcaster.cpp
)
ament_target_dependencies(base_link_broadcaster
  rclcpp
  tf2_ros
  biped_interfaces
)

add_executable(support_state_pub
  src/support_state_pub.cpp
)
ament_target_dependencies(support_state_pub
  rclcpp
  biped_interfaces  # Add biped_interfaces as a dependency
  # Add any additional dependencies here
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executables
install(TARGETS
  base_link_broadcaster
  support_state_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
