#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// PID Controller
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

// Updated ASMC class with enhancements from the paper
class ASMC {
private:
    // Sliding surface parameters
    float lambda_;       // Sliding surface coefficient
    float phi_;          // Boundary layer thickness
    
    // Adaptive gains and parameters
    float alpha_0_;      // Forgetting factor for K0
    float alpha_1_;      // Forgetting factor for K1
    float k_hat_0_;      // Adaptive gain estimate K0
    float k_hat_1_;      // Adaptive gain estimate K1
    float k_min_;        // Minimum allowed gain
    float k_max_;        // Maximum allowed gain
    
    // State tracking
    float prev_s_;       // Previous sliding surface value
    float prev_roll_rate_;
    float filtered_roll_rate_;
    
    // Filtering coefficients
    float alpha_rate_;   // Roll rate LPF coefficient
    float alpha_s_;      // Sliding surface LPF coefficient

    float prev_control_output_ = 0.0f;
    const float MAX_RATE_CHANGE = 0.2f;  // Maximum change per step

    float rate_limit(float new_output, float dt) {
        float max_change = MAX_RATE_CHANGE * dt;
        float limited = std::clamp(
            new_output, 
            prev_control_output_ - max_change,
            prev_control_output_ + max_change
        );
        prev_control_output_ = limited;
        return limited;
    }

public:
    ASMC(float lambda, float phi, float alpha_0, float alpha_1, float k0_init, float k1_init, float k_max = 5.0f)
        : lambda_(lambda), phi_(phi), alpha_0_(alpha_0), alpha_1_(alpha_1),
          k_hat_0_(k0_init), k_hat_1_(k1_init), k_min_(0.01f), k_max_(k_max),
          prev_s_(0.0f), prev_roll_rate_(0.0f), filtered_roll_rate_(0.0f),
          alpha_rate_(0.7f), alpha_s_(0.8f) {}

    float compute(float desired_roll, float current_roll, float roll_rate_raw, float dt) {
        // Apply low-pass filter to roll rate - important for noisy measurements
        filtered_roll_rate_ = alpha_rate_ * roll_rate_raw + (1.0f - alpha_rate_) * prev_roll_rate_;
        prev_roll_rate_ = filtered_roll_rate_;

        // Calculate tracking error and normalize to [-π, π]
        float error = desired_roll - current_roll;
        error = normalizeAngle(error);

        // Calculate sliding surface according to the paper's formulation
        // s = e_dot + lambda * e  (standard form)
        float s_raw = filtered_roll_rate_ + lambda_ * error;

        // Filter the sliding surface to reduce chattering
        float s = alpha_s_ * s_raw + (1.0f - alpha_s_) * prev_s_;
        prev_s_ = s;

        // Equivalent control term - provides nominal control effort
        float u_eq = -lambda_ * filtered_roll_rate_;

        // Adaptive gains update law per the paper
        // The adaptation law increases gain when |s| is large and reduces it when small
        float k_dot_0 = std::abs(s) - alpha_0_ * k_hat_0_;
        float k_dot_1 = std::abs(s) * std::abs(error) - alpha_1_ * k_hat_1_;
        
        // Integrate adaptation laws
        k_hat_0_ += k_dot_0 * dt;
        k_hat_1_ += k_dot_1 * dt;

        // Ensure gains stay within reasonable bounds
        k_hat_0_ = std::clamp(k_hat_0_, k_min_, k_max_);
        k_hat_1_ = std::clamp(k_hat_1_, k_min_, k_max_);

        // Calculate total adaptive gain
        float rho = k_hat_0_ + k_hat_1_ * std::abs(error);

        // Switching control with boundary layer for chattering reduction
        float u_sw = -rho * sat(s, phi_);

        // Total control input: equivalent + switching terms
        float total_input = u_eq + u_sw;

        // Apply rate limiting to reduce sudden control movements
        total_input = rate_limit(total_input, dt);

        // Apply additional safety limits on control output
        return std::clamp(total_input, -1.0f, 1.0f);
    }

    void reset() {
        k_hat_0_ = 0.1f;
        k_hat_1_ = 0.1f;
        prev_s_ = 0.0f;
        prev_roll_rate_ = 0.0f;
        filtered_roll_rate_ = 0.0f;
    }

    // Getters for monitoring
    float get_adaptive_gain_k0() const { return k_hat_0_; }
    float get_adaptive_gain_k1() const { return k_hat_1_; }
    float get_sliding_surface() const { return prev_s_; }
    float get_filtered_roll_rate() const { return filtered_roll_rate_; }

private:
    // Saturation function with smooth boundary layer
    float sat(float value, float threshold) {
        if (std::abs(value) <= threshold) {
            return value / threshold;
        }
        return value > 0 ? 1.0f : -1.0f;
    }
    
    // Angle normalization to [-π, π]
    float normalizeAngle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
};

class AltitudeAttitudeControl : public rclcpp::Node {
public:
    AltitudeAttitudeControl()
        : Node("altitude_attitude_control"), counter_(0),
          asmc_roll_(0.5f, 0.1f, 0.05f, 0.05f, 0.2f, 0.2f, 1.0f),
          pid_pitch_(0.06f, 0.013f, 0.027f),
          pid_yaw_(0.2f, 0.2f, 0.01f),
          pid_altitude_(0.1f, 0.5f, 0.6f) {
        // Publishers and subscriptions
        auto qos_reliable = rclcpp::QoS(1).reliable().transient_local();
        auto qos = rclcpp::QoS(10).best_effort();

        offboard_control_mode_publisher_ = 
            this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
        vehicle_command_publisher_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_reliable);
        servos_publisher_ =
            this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", qos);
        motors_publisher_ =
            this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", qos);

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&AltitudeAttitudeControl::attitude_callback, this, std::placeholders::_1));

        sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            std::bind(&AltitudeAttitudeControl::sensor_callback, this, std::placeholders::_1));

        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            std::bind(&AltitudeAttitudeControl::vehicle_status_callback, this, std::placeholders::_1));

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&AltitudeAttitudeControl::local_pos_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&AltitudeAttitudeControl::timer_callback, this));
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now();
        msg.direct_actuator = true;
        offboard_control_mode_publisher_->publish(msg);
    }

    void engage_offboard_mode() {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = now();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0f; // Mode
        msg.param2 = 6.0f; // Custom mode (offboard)
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
        msg.param1 = 1.0f; // 1 to arm
        msg.param2 = 2989; // Force arm/disarm (override safety checks)
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_publisher_->publish(msg);

        armed_ = true;
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servos_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int counter_;
    ASMC asmc_roll_;
    PID pid_pitch_;
    PID pid_yaw_;
    PID pid_altitude_;

    tf2Scalar desired_roll_ = 0.0f;
    tf2Scalar desired_pitch_ = 0.0f;
    float desired_yaw_rate_ = 0.0f;
    float desired_altitude_ = 50.0f;  // Starting altitude
    tf2Scalar current_roll_ = 0.0f;
    tf2Scalar current_pitch_ = 0.0f;
    tf2Scalar current_yaw_ = 0.0f;
    float current_roll_rate_ = 0.0f;
    float current_pitch_rate_ = 0.0f;
    float current_yaw_rate_ = 0.0f;
    float current_altitude_ = 0.0f;
    float current_vertical_speed_ = 0.0f;

    bool attitude_received_ = false;
    bool gyro_received_ = false;

    uint64_t last_control_time_ = 0;

    enum class ControlState {
        INIT,
        OFFBOARD_REQUESTED,
        ARMING_REQUESTED,
        ACTIVE_CONTROL
    };
    ControlState state_ = ControlState::INIT;
    int init_counter_ = 0;
    bool armed_ = false;
    bool offboard_enabled_ = false;

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
        attitude_received_ = true;
    }

    void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        current_roll_rate_ = msg->gyro_rad[0];
        current_pitch_rate_ = msg->gyro_rad[1];
        current_yaw_rate_ = msg->gyro_rad[2];
        // SensorCombined doesn't have direct altitude - you need another source
        // As a workaround, you could use a separate barometer message or use
        // the vehicle_local_position message
        gyro_received_ = true;
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        offboard_enabled_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }

    void local_pos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_altitude_ = -msg->z;  // NED frame, so -z is altitude
        current_vertical_speed_ = -msg->vz;
    }

    void timer_callback() {
        uint64_t current_time = now();
        float dt = (last_control_time_ > 0) ? 
                  (current_time - last_control_time_) / 1e6f : 0.05f;
        dt = std::clamp(dt, 0.001f, 0.1f);
        last_control_time_ = current_time;

        if (!attitude_received_ || !gyro_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Waiting for complete sensor data before controlling");
            return;
        }

        if (!attitude_received_)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Missing attitude data");
        if (!gyro_received_)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Missing gyro data");

        switch (state_) {
            case ControlState::INIT:
                if (init_counter_++ > 10) {
                    state_ = ControlState::OFFBOARD_REQUESTED;
                    RCLCPP_INFO(this->get_logger(), "Requesting offboard mode");
                    engage_offboard_mode();
                }
                break;

            case ControlState::OFFBOARD_REQUESTED:
                publish_offboard_control_mode();
                if (init_counter_++ > 20) {
                    state_ = ControlState::ARMING_REQUESTED;
                    RCLCPP_INFO(this->get_logger(), "Requesting arming");
                    arm();
                }
                break;
            
            case ControlState::ARMING_REQUESTED:
                if (armed_) {
                    publish_offboard_control_mode();
                    RCLCPP_INFO(this->get_logger(), "Beginning active control");
                    state_ = ControlState::ACTIVE_CONTROL;
                } else if (init_counter_++ > 30) {
                    arm(); // Try again
                }
                break;  // Add this line

            case ControlState::ACTIVE_CONTROL:
                publish_offboard_control_mode();
                
                // Compute control signals
                float roll_cmd = asmc_roll_.compute(desired_roll_, current_roll_, current_roll_rate_, dt);
                float pitch_cmd = pid_pitch_.compute(desired_pitch_ - current_pitch_, dt);
                float yaw_cmd = pid_yaw_.compute(desired_yaw_rate_ - current_yaw_rate_, dt);
                float throttle_cmd = std::clamp(pid_altitude_.compute(desired_altitude_ - current_altitude_, dt), 0.3f, 1.0f);
                
                // Publish motor commands (throttle)
                px4_msgs::msg::ActuatorMotors motors_msg{};
                motors_msg.timestamp = now();
                motors_msg.control[0] = throttle_cmd;
                motors_publisher_->publish(motors_msg);
                
                // Publish servo commands (control surfaces)
                px4_msgs::msg::ActuatorServos servos_msg{};
                servos_msg.timestamp = now();
                servos_msg.control[0] = -roll_cmd;  // Left aileron
                servos_msg.control[1] = roll_cmd;   // Right aileron
                servos_msg.control[2] = pitch_cmd;  // Elevator
                servos_msg.control[3] = yaw_cmd;    // Rudder
                servos_publisher_->publish(servos_msg);
                
                // Log adaptive gains
                RCLCPP_INFO(this->get_logger(), 
                            "ASMC: K0=%.3f, K1=%.3f, S=%.3f, Alt: %.1f->%.1f", 
                            asmc_roll_.get_adaptive_gain_k0(),
                            asmc_roll_.get_adaptive_gain_k1(),
                            asmc_roll_.get_sliding_surface(),
                            current_altitude_,
                            desired_altitude_);
                break;
        }
    }

    // Helper function to get timestamp in microseconds
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