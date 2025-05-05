#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <cmath>
#include <algorithm>
#include <array>  // Needed for msg->q

using namespace std::chrono_literals;

struct PID {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;

    PID(float p, float i, float d) : kp(p), ki(i), kd(d), prev_error(0.0f), integral(0.0f) {}

    float compute(float error, float dt) {
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

class AdvancedPlaneControl : public rclcpp::Node {
public:
    AdvancedPlaneControl()
    : Node("attitude_pid_control"), counter_(0),
      pid_roll_(0.8f, 0.01f, 0.05f),
      pid_pitch_(0.8f, 0.01f, 0.05f),
      pid_yaw_(0.5f, 0.01f, 0.03f)
    {
        auto qos = rclcpp::QoS(10);

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);
        motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", qos);
        servos_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", qos);

        odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&AdvancedPlaneControl::odometry_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&AdvancedPlaneControl::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servos_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_;

    PID pid_roll_;
    PID pid_pitch_;
    PID pid_yaw_;

    float desired_roll_ = M_PI / 6.0f;
    float desired_pitch_ = M_PI / 6.0f;
    float desired_yaw_ = 0.5f;

    float current_roll_ = 0.0f;
    float current_pitch_ = 0.0f;
    float current_yaw_ = 0.0f;

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        float q0 = msg->q[0];
        float q1 = msg->q[1];
        float q2 = msg->q[2];
        float q3 = msg->q[3];

        current_roll_  = std::atan2(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
        current_pitch_ = std::asin(std::clamp(2.0f * (q0*q2 - q3*q1), -1.0f, 1.0f));
        current_yaw_   = std::atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));

        RCLCPP_INFO(this->get_logger(), "Current [roll: %.2f, pitch: %.2f, yaw: %.2f]", current_roll_, current_pitch_, current_yaw_);
    }

    void timer_callback() {
        publish_offboard_control_mode();
        publish_actuator_values();

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
        msg.direct_actuator = true;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_actuator_values() {
        float dt = 0.1f;

        float roll_error = desired_roll_ - current_roll_;
        float pitch_error = desired_pitch_ - current_pitch_;
        float yaw_error = desired_yaw_ - current_yaw_;

        float roll_cmd = std::clamp(pid_roll_.compute(roll_error, dt), -1.0f, 1.0f);
        float pitch_cmd = std::clamp(pid_pitch_.compute(pitch_error, dt), -1.0f, 1.0f);
        float yaw_cmd = std::clamp(pid_yaw_.compute(yaw_error, dt), -1.0f, 1.0f);

        auto now_us = this->get_clock()->now().nanoseconds() / 1000;

        px4_msgs::msg::ActuatorMotors motors_msg{};
        motors_msg.timestamp = now_us;
        motors_msg.timestamp_sample = now_us;
        motors_msg.control[0] = 0.8f;  // Throttle
        motors_publisher_->publish(motors_msg);

        px4_msgs::msg::ActuatorServos servos_msg{};
        servos_msg.timestamp = now_us;
        servos_msg.control[0] = roll_cmd;   // Left aileron
        servos_msg.control[1] = -roll_cmd;  // Right aileron
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
        msg.param1 = 1.0;
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
