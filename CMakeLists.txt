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
add_executable(pubtest src/pubtest.cpp)
add_executable(actuator_test src/actuator_test.cpp) 
target_link_libraries(pubtest rclcpp::rclcpp)  
target_link_libraries(actuator_test rclcpp::rclcpp)  
ament_target_dependencies(pubtest rclcpp px4_msgs)
ament_target_dependencies(actuator_test rclcpp px4_msgs)

# Ensure C++17 compatibility
target_compile_features(pubtest PUBLIC c_std_99 cxx_std_17)
target_compile_features(actuator_test PUBLIC c_std_99 cxx_std_17)

# Install the executable
install(TARGETS pubtest
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS actuator_test
  DESTINATION lib/${PROJECT_NAME})

# Final ROS 2 package declaration
ament_package()
