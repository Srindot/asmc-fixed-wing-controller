#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class MinimalOffboardControl : public rclcpp::Node {
public:
    MinimalOffboardControl() : Node("minimal_offboard_control"), counter_(0) {
        // Set QoS profile - critical for PX4 communication
        auto qos = rclcpp::QoS(1)
            .best_effort()
            .transient_local();
        
        // Publishers
        offboard_control_mode_publisher_ = 
            this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        vehicle_command_publisher_ = 
            this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);
        
        // Timer - 100ms (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&MinimalOffboardControl::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

    void timer_callback() {
        // Always publish offboard control mode
        publish_offboard_control_mode();
        
        // After 10 iterations (1 second), engage offboard and arm
        if (counter_ == 10) {
            engage_offboard_mode();
        }
        
        if (counter_ == 15) {
            arm();
        }
        
        counter_++;
    }
    
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;  // Using position control like Python code
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_publisher_->publish(msg);
    }
    
    void engage_offboard_mode() {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0;  // Custom mode
        msg.param2 = 6.0;  // Offboard mode
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Offboard mode requested");
    }
    
    void arm() {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0;  // 1.0 = arm
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalOffboardControl>());
    rclcpp::shutdown();
    return 0;
}
