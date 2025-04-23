cmake_minimum_required(VERSION 3.8)
project(offboard_control_direct_actuator) # Or your preferred project name

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add executable for your offboard control node
add_executable(test  # Or your executable name
  src/control_cpp/test.cpp      # Your main C++ source file
)

ament_target_dependencies(test rclcpp px4_msgs)

install(TARGETS test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

############
# Install ##
############

ament_package()