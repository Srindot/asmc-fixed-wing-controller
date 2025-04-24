#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <chrono>

using namespace std::chrono_literals;

class FixedWingControl : public rclcpp::Node {
public:
    FixedWingControl() : Node("fixed_wing_control") {
        // Publishers for control commands
        rates_publisher_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);
        thrust_publisher_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 10);
        command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        // Timer for control updates
        timer_ = this->create_wall_timer(500ms, std::bind(&FixedWingControl::publish_control_commands, this));

        activate_offboard_mode();
        arm_vehicle();
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void activate_offboard_mode() {
        px4_msgs::msg::OffboardControlMode offboard_msg{};
        offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_msg.body_rate = true;

        offboard_publisher_->publish(offboard_msg);
        RCLCPP_INFO(this->get_logger(), "Offboard control mode activated.");
    }

    void arm_vehicle() {
        px4_msgs::msg::VehicleCommand arm_msg{};
        arm_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        arm_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        arm_msg.param1 = 1.0f;
        arm_msg.target_system = 1;
        arm_msg.target_component = 1;
        arm_msg.source_system = 1;
        arm_msg.source_component = 1;
        arm_msg.from_external = true;

        command_publisher_->publish(arm_msg);
        RCLCPP_INFO(this->get_logger(), "Sent ARM command to vehicle.");
    }

    void publish_control_commands() {
        // Publish body angular rate setpoints
        px4_msgs::msg::VehicleRatesSetpoint rates_msg{};
        rates_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        rates_msg.roll = 0.05;
        rates_msg.pitch = -0.02;
        rates_msg.yaw = 0.03;
        rates_msg.thrust_body[0] = 0.1;
        rates_msg.thrust_body[1] = 0.0;
        rates_msg.thrust_body[2] = -0.05;
        rates_msg.reset_integral = false;
        rates_publisher_->publish(rates_msg);

        RCLCPP_INFO(this->get_logger(), "Published Vehicle Rates: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", rates_msg.roll, rates_msg.pitch, rates_msg.yaw);

        // Publish thrust setpoints
        px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
        thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        thrust_msg.xyz[0] = 0.3;
        thrust_msg.xyz[1] = 0.0;
        thrust_msg.xyz[2] = -0.2;

        thrust_publisher_->publish(thrust_msg);

        RCLCPP_INFO(this->get_logger(), "Published Thrust Setpoint: X=%.2f, Y=%.2f, Z=%.2f", thrust_msg.xyz[0], thrust_msg.xyz[1], thrust_msg.xyz[2]);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixedWingControl>());
    rclcpp::shutdown();
    return 0;
}
