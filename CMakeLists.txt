cmake_minimum_required(VERSION 3.8)
project(offboard_control_direct_actuator) # You can choose a different project name if you prefer

find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(include)

add_executable(test  # The name of your executable will be 'test'
  src/test.cpp      # Specifies your C++ source file
)

ament_target_dependencies(test rclcpp px4_msgs)

install(TARGETS test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()