#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hello_world_node");

  RCLCPP_INFO(node->get_logger(), "Hello, World!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
