#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <chrono>
#include <vector>
#include <cmath>
#include <functional>

namespace my_offboard_control {

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl()
  : Node("offboard_control")
  {
    // Configure QoS profile
    rclcpp::QoS qos_profile(1);
    qos_profile.reliability(rclcpp::QoSInitialization::ReliabilityPolicy::BEST_EFFORT);
    qos_profile.durability(rclcpp::QoSInitialization::DurabilityPolicy::TRANSIENT_LOCAL);
    qos_profile.history(rclcpp::QoSInitialization::HistoryPolicy::KEEP_LAST);

    // Create publishers
    offboard_control_mode_publisher_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
    trajectory_setpoint_publisher_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
    vehicle_command_publisher_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);

    // Create subscribers
    vehicle_local_position_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos_profile,
      std::bind(&OffboardControl::vehicleLocalPositionCallback, this, std::placeholders::_1));
    vehicle_status_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos_profile,
      std::bind(&OffboardControl::vehicleStatusCallback, this, std::placeholders::_1));

    // Set Waypoints
    waypoints_ = {{1000.0f, 10000.0f, -30.0f}};

    // Initialize variables
    waypoint_ = waypoints_[0];
    offboard_setpoint_counter_ = 0;
    waypoint_change_counter_ = 0;
    delay_at_waypoint_counter_ = 0;
    vehicle_local_position_.x = 0.0f;
    vehicle_local_position_.y = 0.0f;
    vehicle_local_position_.z = 0.0f;

    // Create a timer to publish control commands
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), // 0.1 seconds
      std::bind(&OffboardControl::timerCallback, this));
  }

private:
  // Publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

  // Waypoints
  std::vector<std::array<float, 3>> waypoints_;
  std::array<float, 3> waypoint_;
  int offboard_setpoint_counter_;
  int waypoint_change_counter_;
  int delay_at_waypoint_counter_;
  px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
  px4_msgs::msg::VehicleStatus vehicle_status_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void arm();
  void disarm();
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void engageOffboardMode();
  void publishOffboardControlMode();
  void publishTrajectorySetpoint();
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
  void timerCallback();
  void changeWaypoint();
  bool checkPos();
};

void OffboardControl::arm()
{
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControl::disarm()
{
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
  RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControl::vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  vehicle_local_position_ = *msg;
  RCLCPP_INFO(this->get_logger(),
              "Vehicle Position: x=%.2f, y=%.2f, z=%.2f",
              msg->x, msg->y, msg->z);
}

void OffboardControl::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  vehicle_status_ = *msg;
}

void OffboardControl::engageOffboardMode()
{
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  RCLCPP_INFO(this->get_logger(), "Switching to offboard mode");
}

void OffboardControl::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.direct_actuator = true; // Setting direct actuator control
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publishTrajectorySetpoint()
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position = waypoint_;
  msg.yaw = 0.0f;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
              "Publishing trajectory setpoint: x=%.2f, y=%.2f, z=%.2f",
              waypoint_[0], waypoint_[1], waypoint_[2]);
}

void OffboardControl::publishVehicleCommand(uint16_t command, float param1, float param2)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.command = command;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

void OffboardControl::timerCallback()
{
  changeWaypoint();
  publishOffboardControlMode();
  // We are now in direct actuator control, so we won't publish trajectory setpoints
  // publishTrajectorySetpoint();

  if (offboard_setpoint_counter_ == 10) {
    engageOffboardMode();
    arm();
  }

  if (offboard_setpoint_counter_ < 11) {
    offboard_setpoint_counter_++;
  }
}

void OffboardControl::changeWaypoint()
{
  if (checkPos()) {
    if (waypoint_change_counter_ < waypoints_.size() - 1) {
      waypoint_change_counter_++;
      waypoint_ = waypoints_[waypoint_change_counter_];
    }
  }
}

bool OffboardControl::checkPos()
{
  float tolerance = 0.2f;

  if (std::abs(vehicle_local_position_.x - waypoint_[0]) < tolerance &&
      std::abs(vehicle_local_position_.y - waypoint_[1]) < tolerance &&
      std::abs(vehicle_local_position_.z - waypoint_[2]) < tolerance) {
    if (delay_at_waypoint_counter_ < 3) {
      delay_at_waypoint_counter_++;
    } else {
      delay_at_waypoint_counter_ = 0;
      return true;
    }
  } else {
    return false;
  }
  return false; // Added to satisfy the compiler
}

} // namespace my_offboard_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_offboard_control::OffboardControl>());
  rclcpp::shutdown();
  return 0;
}