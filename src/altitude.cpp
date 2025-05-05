#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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
    float integral_limit;

    PID(float p, float i, float d) 
        : kp(p), ki(i), kd(d), prev_error(0.0f), integral(0.0f), integral_limit(1.0f) {}

    float compute(float error, float dt) {
        integral += error * dt;
        integral = std::clamp(integral, -integral_limit, integral_limit);
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

class AltitudeAttitudeControl : public rclcpp::Node {
public:
    AltitudeAttitudeControl()
    : Node("altitude_attitude_control"), counter_(0),
      pid_roll_(0.1f, 0.1f, 0.001f),
      pid_pitch_(0.001f, 0.1f, 0.05f),
      pid_yaw_(0.2f, 0.2f, 0.01f),
      pid_altitude_(0.001f, 0.1f, 0.01f)  // Tuned for altitude control
    {
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
            std::bind(&AltitudeAttitudeControl::attitude_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&AltitudeAttitudeControl::odometry_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&AltitudeAttitudeControl::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servos_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_;
    PID pid_roll_;
    PID pid_pitch_;
    PID pid_yaw_;
    PID pid_altitude_;

    tf2Scalar desired_roll_ = 0.0f;
    tf2Scalar desired_pitch_ = 0.0f;
    tf2Scalar desired_yaw_ = 3 * M_PI / 2;

    float desired_altitude_ = 50.0f;  // Target altitude in meters
    float current_altitude_ = 0.0f;

    tf2Scalar current_roll_ = 0.0f;
    tf2Scalar current_pitch_ = 0.0f;
    tf2Scalar current_yaw_ = 0.0f;

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
        if (current_yaw_ < 0) current_yaw_ += 2.0 * M_PI;
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        current_altitude_ = -msg->position[2];  // Negative in NED
    }

    void timer_callback() {
        publish_offboard_control_mode();

        if (counter_ == 10) engage_offboard_mode();
        if (counter_ == 15) arm();

        publish_actuator_values();
        counter_++;
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now();
        msg.direct_actuator = true;
        offboard_control_mode_publisher_->publish(msg);
    }



    void publish_actuator_values() {
        float dt = 0.1f;
    
        float roll_cmd = std::clamp(pid_roll_.compute(desired_roll_ - current_roll_, dt), -1.0f, 1.0f);
        float yaw_cmd  = std::clamp(pid_yaw_.compute(desired_yaw_ - current_yaw_, dt), -1.0f, 1.0f);
    
        float altitude_error = desired_altitude_ - current_altitude_;
        float throttle = std::clamp(pid_altitude_.compute(altitude_error, dt), 0.3f, 1.0f);
    
        // Pitch down slightly when below desired altitude
        float altitude_pitch_correction = std::clamp(0.2f * altitude_error, -0.1f, 0.1f);
        float pitch_cmd = std::clamp(pid_pitch_.compute(altitude_pitch_correction - current_pitch_, dt), -1.0f, 1.0f);
    
        auto now_us = now();
    
        // Publish thrust
        px4_msgs::msg::ActuatorMotors motors_msg{};
        motors_msg.timestamp = now_us;
        motors_msg.timestamp_sample = now_us;
        motors_msg.control[0] = throttle;
        motors_publisher_->publish(motors_msg);
    
        // Publish attitude commands
        px4_msgs::msg::ActuatorServos servos_msg{};
        servos_msg.timestamp = now_us;
        servos_msg.control[0] = -roll_cmd;
        servos_msg.control[1] = roll_cmd;
        servos_msg.control[2] = pitch_cmd;
        servos_msg.control[3] = yaw_cmd;
        servos_publisher_->publish(servos_msg);
    
        RCLCPP_INFO(this->get_logger(),
            "Alt tgt: %.2f | Cur: %.2f | Thr: %.2f | Cmd: %.2f",
            desired_altitude_, current_altitude_, throttle,
            pitch_cmd);
    }
    

    void engage_offboard_mode() {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = now();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0;
        msg.param2 = 6.0;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_publisher_->publish(msg);
    }

    void arm() {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = now();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0;
        msg.param2 = 2989;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_publisher_->publish(msg);
    }

    uint64_t now() {
        return this->get_clock()->now().nanoseconds() / 1000;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AltitudeAttitudeControl>());
    rclcpp::shutdown();
    return 0;
}
