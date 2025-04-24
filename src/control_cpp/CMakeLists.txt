cmake_minimum_required(VERSION 3.8)
project(control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable node (renamed from control_node to hello_world_node)
add_executable(hello_world_node src/asdf.cpp)
target_link_libraries(hello_world_node rclcpp::rclcpp)

# Link dependencies
ament_target_dependencies(hello_world_node rclcpp)

# Install the node executable
install(TARGETS hello_world_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
