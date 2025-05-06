#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

struct PID {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float integral_limit;  // Added to prevent windup

    PID(float p, float i, float d) 
        : kp(p), ki(i), kd(d), prev_error(0.0f), integral(0.0f), integral_limit(1.0f) {}

    float compute(float error, float dt) {
        integral += error * dt;
        // Anti-windup protection
        integral = std::clamp(integral, -integral_limit, integral_limit);
        
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

class AdvancedPlaneControl : public rclcpp::Node {
public:
    AdvancedPlaneControl()
    : Node("attitude_pid_control"), counter_(0),
      pid_roll_(0.08f, 0.1f, 0.001f),
      pid_pitch_(0.06f, 0.013f, 0.01f),
      pid_yaw_(0.08f, 0.2f, 0.01f)
    {
        // Use reliable QoS for critical commands
        auto qos_reliable = rclcpp::QoS(1).reliable().transient_local();
        auto qos = rclcpp::QoS(10).best_effort();

        offboard_control_mode_publisher_ = 
            this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        vehicle_command_publisher_ = 
            this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_reliable);
        motors_publisher_ = 
            this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", qos);
        servos_publisher_ = 
            this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", qos);

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&AdvancedPlaneControl::attitude_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&AdvancedPlaneControl::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servos_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_;

    PID pid_roll_;
    PID pid_pitch_;
    PID pid_yaw_;

    tf2Scalar desired_roll_ = 0.0f;   // Start with level flight
    tf2Scalar desired_pitch_ = -0.0f;  // Start with level flight
    tf2Scalar desired_yaw_ = 0.0;    // Start with current heading

    tf2Scalar current_roll_ = 0.0f;
    tf2Scalar current_pitch_ = 0.0f;
    tf2Scalar current_yaw_ = 0.0f;

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        // Convert quaternion to Euler angles (roll, pitch, yaw)
        tf2::Quaternion q(
            msg->q[1],  // x
            msg->q[2],  // y
            msg->q[3],  // z
            msg->q[0]   // w (real part first in PX4)
        );

        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);

            // Normalize yaw to [0, 2π] range
        if (current_yaw_ < 0) {
            current_yaw_ += 2.0 * M_PI;
        }
        
        // Debug output
        RCLCPP_DEBUG(this->get_logger(), "Current attitude: Roll=%.2f, Pitch=%.2f, Yaw=%.2f", 
                   current_roll_, current_pitch_, current_yaw_);
    }

    void timer_callback() {
        publish_offboard_control_mode();
        
        // After sufficient time, engage offboard and arm
        if (counter_ == 10) {
            engage_offboard_mode();
        }
        
        if (counter_ == 15) {
            arm();
        }
        
        // // After arming, set desired angles to something non-zero
        // if (counter_ == 30) {
        //     desired_roll_ = M_PI / 18.0f;   // 10 degrees in radians
        //     desired_pitch_ = M_PI / 18.0f;  // 10 degrees in radians
        // }
        
        publish_actuator_values();
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
        msg.direct_actuator = true;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_actuator_values() {
        float dt = 0.1f;

        // Compute error for each axis
        float roll_error = desired_roll_ - current_roll_;
        float pitch_error = desired_pitch_ - current_pitch_;
        float yaw_error = desired_yaw_ - current_yaw_;

        // Use PID to compute control outputs for each axis
        float roll_cmd = std::clamp(pid_roll_.compute(roll_error, dt), -1.0f, 1.0f);
        float pitch_cmd = std::clamp(pid_pitch_.compute(pitch_error, dt), -1.0f, 1.0f);
        float yaw_cmd = std::clamp(pid_yaw_.compute(yaw_error, dt), -1.0f, 1.0f);

        auto now_us = this->get_clock()->now().nanoseconds() / 1000;

        // Publish motor control message (Throttle)
        px4_msgs::msg::ActuatorMotors motors_msg{};
        motors_msg.timestamp = now_us;
        motors_msg.timestamp_sample = now_us;
        motors_msg.control[0] = 0.8f;  // Throttle - critical to prevent ESC failure
        motors_publisher_->publish(motors_msg);

        // Publish servo control message (Ailerons, Elevator, Rudder)
        px4_msgs::msg::ActuatorServos servos_msg{};
        servos_msg.timestamp = now_us;
        servos_msg.control[0] = -roll_cmd;   // Left aileron
        servos_msg.control[1] = roll_cmd;  // Right aileron
        servos_msg.control[2] = pitch_cmd;  // Elevator
        servos_msg.control[3] = yaw_cmd;    // Rudder
        servos_publisher_->publish(servos_msg);

        RCLCPP_INFO(this->get_logger(), "Desired [%.2f, %.2f, %.2f] | Current [%.2f, %.2f, %.2f] | Output [%.2f, %.2f, %.2f]",
                    desired_roll_, desired_pitch_, desired_yaw_,
                    current_roll_, current_pitch_, current_yaw_,
                    roll_cmd, pitch_cmd, yaw_cmd);
    }

    void engage_offboard_mode() {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0;
        msg.param2 = 6.0;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
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
        msg.param2 = 2989; // Force arm - overrides safety checks
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
