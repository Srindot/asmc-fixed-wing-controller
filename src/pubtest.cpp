#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <chrono>

using namespace std::chrono_literals;

class PIDControlPublisher : public rclcpp::Node {
public:
    PIDControlPublisher() : Node("pid_control_publisher") {
        // Publishers for vehicle rates, thrust setpoints, and vehicle commands
        rates_publisher_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);
        thrust_publisher_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);
        command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Timer to publish control commands periodically
        timer_ = this->create_wall_timer(500ms, std::bind(&PIDControlPublisher::publish_control_commands, this));

        // Arm the vehicle
        arm_vehicle();
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void arm_vehicle() {
        px4_msgs::msg::VehicleCommand command_msg{};
        command_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        command_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        command_msg.param1 = 1.0f;  // 1.0 = Arm, 0.0 = Disarm
        command_msg.target_system = 1;
        command_msg.target_component = 1;
        command_msg.source_system = 1;
        command_msg.source_component = 1;
        command_msg.from_external = true;
        
        command_publisher_->publish(command_msg);
        RCLCPP_INFO(this->get_logger(), "Sent ARM command to vehicle.");
    }

    void publish_control_commands() {
        // Publish body angular rate setpoints
        px4_msgs::msg::VehicleRatesSetpoint rates_msg{};
        rates_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        rates_msg.roll = 0.05;    // Arbitrary roll rate [rad/s]
        rates_msg.pitch = -0.02;  // Arbitrary pitch rate [rad/s]
        rates_msg.yaw = 0.03;     // Arbitrary yaw rate [rad/s]
        rates_msg.thrust_body[0] = 0.1; // Small forward thrust
        rates_msg.thrust_body[1] = 0.0;
        rates_msg.thrust_body[2] = -0.05; // Small downward thrust
        rates_msg.reset_integral = false;
        rates_publisher_->publish(rates_msg);

        RCLCPP_INFO(this->get_logger(), "Published Vehicle Rates: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", rates_msg.roll, rates_msg.pitch, rates_msg.yaw);

        // Publish thrust setpoints
        px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
        thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        thrust_msg.xyz[0] = 0.3;  // Arbitrary thrust in X direction
        thrust_msg.xyz[1] = 0.0;  // No thrust in Y direction
        thrust_msg.xyz[2] = -0.2; // Negative for altitude control

        thrust_publisher_->publish(thrust_msg);

        RCLCPP_INFO(this->get_logger(), "Published Thrust Setpoint: X=%.2f, Y=%.2f, Z=%.2f", thrust_msg.xyz[0], thrust_msg.xyz[1], thrust_msg.xyz[2]);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControlPublisher>());
    rclcpp::shutdown();
    return 0;
}
