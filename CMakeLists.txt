cmake_minimum_required(VERSION 3.8)
project(fixed_wing_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # ROS 2 core library
find_package(px4_msgs REQUIRED)  # PX4 message definitions

# Define the executable
add_executable(controller src/controller.cpp)
target_link_libraries(controller rclcpp::rclcpp)  # REMOVE `px4_msgs::px4_msgs`
ament_target_dependencies(controller rclcpp px4_msgs)  # `px4_msgs` must stay here

# Ensure C++17 compatibility
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)

# Install the executable
install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

# Final ROS 2 package declaration
ament_package()
