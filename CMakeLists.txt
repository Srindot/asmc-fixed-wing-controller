cmake_minimum_required(VERSION 3.8)
project(fixed_wing_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable C++20 to support designated initializers
add_compile_options(-std=c++20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # ROS 2 core library
find_package(px4_msgs REQUIRED)  # PX4 message definitions
find_package(tf2 REQUIRED)  # Added for quaternion to Euler conversion
find_package(tf2_ros REQUIRED)  # Added for TF2 ROS integration

# Define the executable
add_executable(pubtest src/pubtest.cpp)
add_executable(actuator_test src/actuator_test.cpp) 
add_executable(pid src/pid.cpp) 
add_executable(attitude src/attitude.cpp)
add_executable(altitude src/altitude.cpp)
add_executable(display_altitude src/display_altitude.cpp)
add_executable(asmc src/asmc.cpp)

# Link libraries
target_link_libraries(pubtest rclcpp::rclcpp)  
target_link_libraries(actuator_test rclcpp::rclcpp)  
target_link_libraries(pid rclcpp::rclcpp)  
target_link_libraries(attitude rclcpp::rclcpp ${tf2_LIBRARIES})  
target_link_libraries(altitude rclcpp::rclcpp ${tf2_LIBRARIES})  
target_link_libraries(display_altitude rclcpp::rclcpp ${tf2_LIBRARIES})  
target_link_libraries(asmc rclcpp::rclcpp ${tf2_LIBRARIES})  

# Declare dependencies
ament_target_dependencies(pubtest rclcpp px4_msgs)
ament_target_dependencies(actuator_test rclcpp px4_msgs)
ament_target_dependencies(pid rclcpp px4_msgs tf2 tf2_ros)
ament_target_dependencies(attitude rclcpp px4_msgs tf2 tf2_ros)
ament_target_dependencies(altitude rclcpp px4_msgs tf2 tf2_ros)
ament_target_dependencies(display_altitude rclcpp px4_msgs tf2 tf2_ros)
ament_target_dependencies(asmc rclcpp px4_msgs tf2 tf2_ros)

# Install the executables
install(TARGETS 
  pubtest
  actuator_test
  pid
  attitude
  altitude
  display_altitude
  asmc
  DESTINATION lib/${PROJECT_NAME})

# Final ROS 2 package declaration
ament_package()
