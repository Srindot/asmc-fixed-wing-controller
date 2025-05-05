#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>  // For motor control
#include <px4_msgs/msg/actuator_servos.hpp>  // For servo control

using namespace std::chrono_literals;

class AdvancedPlaneControl : public rclcpp::Node {
public:
    AdvancedPlaneControl() : Node("advanced_plane_control"), counter_(0) {
        // Set QoS profile
        auto qos = rclcpp::QoS(1)
            .best_effort()
            .transient_local();
        
        // Publishers
        offboard_control_mode_publisher_ = 
            this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        vehicle_command_publisher_ = 
            this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);
        motors_publisher_ = 
            this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", qos);
        servos_publisher_ = 
            this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", qos);
        
        // Timer - 100ms (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&AdvancedPlaneControl::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servos_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

    void timer_callback() {
        // Always publish offboard control mode
        publish_offboard_control_mode();
        
        // Publish actuator values
        publish_actuator_values();
        
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
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.direct_actuator = true;  // Direct actuator control
        offboard_control_mode_publisher_->publish(msg);
    }
    
    void publish_actuator_values() {
        // Publish motor values
        px4_msgs::msg::ActuatorMotors motors_msg{};
        motors_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        motors_msg.timestamp_sample = motors_msg.timestamp;
        motors_msg.reversible_flags = 0;  // No reversible motors
    
        // Assign arbitrary values to each of the 12 motors
        motors_msg.control[0] = 0.9; // Thrust
        motors_msg.control[1] = 0.0;
        motors_msg.control[2] = 0.0;
        motors_msg.control[3] = 0.0;
        motors_msg.control[4] = 0.0;
        motors_msg.control[5] = 0.0;
        motors_msg.control[6] = 0.0;
        motors_msg.control[7] = 0.0;
        motors_msg.control[8] = 0.0;
        motors_msg.control[9] = 0.0;
        motors_msg.control[10] = 0.0;
        motors_msg.control[11] = 0.0;
        // If NUM_CONTROLS > 12, set the rest as needed
    
        motors_publisher_->publish(motors_msg);
    
        // Publish servo values
        px4_msgs::msg::ActuatorServos servos_msg{};
        servos_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
        // Assign arbitrary values to each of the 8 servos
        servos_msg.control[0] =  0.0; // Left aileron
        servos_msg.control[1] = -0.0; // Right aileron (Right) Positive Right Turn, if this is negative left turn 
        servos_msg.control[2] =  0.2; // Elevator Positive PUll Up
        servos_msg.control[3] =  0.0; // Rudder Positive Right Turn 
        servos_msg.control[4] =  -0.4; // Left Flap (shoud be negative ig) positve pulls up
        servos_msg.control[5] =  -0.4; // Right Flap (should be same as the left one and be negative ig)
        servos_msg.control[6] =  0.0; // Nothing
        servos_msg.control[7] =  0.0; // Nothing
        // If NUM_CONTROLS > 8, set the rest as needed
    
        servos_publisher_->publish(servos_msg);
    
        RCLCPP_INFO(this->get_logger(), "Offboard mode requested");
        RCLCPP_INFO(this->get_logger(), "Number of actuator servos: %d", px4_msgs::msg::ActuatorServos::NUM_CONTROLS);
        RCLCPP_INFO(this->get_logger(), "Number of actuator Motors: %d", px4_msgs::msg::ActuatorMotors::NUM_CONTROLS);
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
    rclcpp::spin(std::make_shared<AdvancedPlaneControl>());
    rclcpp::shutdown();
    return 0;
}
