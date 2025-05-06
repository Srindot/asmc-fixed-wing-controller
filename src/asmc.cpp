#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>  // Added for gyro data
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

// Improved ASMC Controller for Roll
class ASMC {
    private:
        // ASMC Parameters
        float lambda_;       // Sliding surface coefficient
        float phi_;          // Boundary layer thickness
        
        // Adaptation parameters
        float alpha_0_;      // Forgetting factor for K0
        float alpha_1_;      // Forgetting factor for K1
        float k_hat_0_;      // Adaptive gain estimate K0
        float k_hat_1_;      // Adaptive gain estimate K1
        float k_max_;        // Maximum allowed gain
        
        // Previous values for filtering
        float prev_error_;
        float prev_s_;       // Previous sliding surface value
        float prev_roll_rate_;
        
        // Filter coefficients
        float alpha_rate_;   // Roll rate LPF coefficient (0-1)
        float alpha_s_;      // Sliding surface LPF coefficient (0-1)
        
    public:
        ASMC(float lambda, float phi, float alpha_0, float alpha_1, float k0_init, float k1_init, float k_max = 5.0f)
            : lambda_(lambda), 
              phi_(phi), 
              alpha_0_(alpha_0),
              alpha_1_(alpha_1),
              k_hat_0_(k0_init),
              k_hat_1_(k1_init),
              k_max_(k_max),
              prev_error_(0.0f),
              prev_s_(0.0f),
              prev_roll_rate_(0.0f),
              alpha_rate_(0.7f),
              alpha_s_(0.8f) {}
        
        float compute(float desired_roll, float current_roll, float roll_rate_raw, float dt) {
            // Apply low-pass filter to roll rate to reduce noise sensitivity
            float roll_rate = alpha_rate_ * roll_rate_raw + (1.0f - alpha_rate_) * prev_roll_rate_;
            prev_roll_rate_ = roll_rate;
            
            // Calculate tracking error
            float error = desired_roll - current_roll;
            
            // Calculate sliding surface (s = ė + λe) 
            float s_raw = -roll_rate + lambda_ * error;
            
            // Filter the sliding surface for smoother control
            float s = alpha_s_ * s_raw + (1.0f - alpha_s_) * prev_s_;
            prev_s_ = s;
            
            // Equivalent control term (as per the theory)
            float u_eq = -lambda_ * s;
            
            // Adaptive gains update law with forgetting factors
            // K̂0(t) = |s(t)| - α0 K̂0(t)
            float k_dot_0 = std::abs(s) - alpha_0_ * k_hat_0_;
            // K̂1(t) = |s(t)||q(t)| - α1 K̂1(t)
            // Here q(t) corresponds to the error state
            float k_dot_1 = std::abs(s) * std::abs(error) - alpha_1_ * k_hat_1_;
            
            // Update adaptive gains
            k_hat_0_ += k_dot_0 * dt;
            k_hat_1_ += k_dot_1 * dt;
            
            // Maintain gains within reasonable bounds
            k_hat_0_ = std::max(0.0f, k_hat_0_);
            k_hat_0_ = std::min(k_hat_0_, k_max_);
            k_hat_1_ = std::max(0.0f, k_hat_1_);
            k_hat_1_ = std::min(k_hat_1_, k_max_);
            
            // Calculate ρ(t) = K̂0(t) + K̂1(t)|q(t)| per theory
            float rho = k_hat_0_ + k_hat_1_ * std::abs(error);
            
            // Switching control with boundary layer
            // τ(t) = -Λs(t) - ρ(t)sgn(s(t))
            float u_sw = -rho * sat(s, phi_);
            
            // Store previous values
            prev_error_ = error;
            
            // Total control input
            float total_input = u_eq + u_sw;
            
            // Apply output limiting for safety
            return std::clamp(total_input, -1.0f, 1.0f);
        }
    
        // Reset controller state
        void reset() {
            k_hat_0_ = 0.1f;  // Initial value
            k_hat_1_ = 0.1f;  // Initial value
            prev_error_ = 0.0f;
            prev_s_ = 0.0f;
            prev_roll_rate_ = 0.0f;
        }
    
        // Getters for adaptive gains
        float get_adaptive_gain_k0() const { return k_hat_0_; }
        float get_adaptive_gain_k1() const { return k_hat_1_; }
        float get_sliding_surface() const { return prev_s_; }
        
    private:
        // Saturation function with boundary layer
        float sat(float value, float threshold) {
            if (std::abs(value) <= threshold) {
                return value / threshold;  // Linear region inside boundary
            }
            return value > 0 ? 1.0f : -1.0f;  // Saturated outside boundary
        }
};

class AltitudeAttitudeControl : public rclcpp::Node {
public:
    AltitudeAttitudeControl()
    : Node("altitude_attitude_control"), counter_(0),
  // ASMC for roll with corrected parameters for fixed-wing
  // Format: lambda, phi, alpha_0, alpha_1, k0_init, k1_init, k_max
  asmc_roll_(0.8f, 0.15f, 0.01f, 0.01f, 0.1f, 0.1f, 3.0f),
  pid_pitch_(0.06f, 0.013f, 0.027f),
  pid_yaw_(0.2f, 0.2f, 0.01f),
  pid_altitude_(0.1f, 0.5f, 0.6f)
    {

        // In the constructor:
        // Declare parameters with default values
        this->declare_parameter("roll_lambda", 0.8f);
        this->declare_parameter("roll_phi", 0.15f);
        this->declare_parameter("roll_alpha_0", 0.01f);
        this->declare_parameter("roll_alpha_1", 0.01f);
        this->declare_parameter("roll_k0_init", 0.1f);
        this->declare_parameter("roll_k1_init", 0.1f);
        this->declare_parameter("roll_k_max", 3.0f);
        
        // Load parameters if specified
        load_parameters();

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
            
        // Add subscription for sensor data to get direct gyro measurements
        sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            std::bind(&AltitudeAttitudeControl::sensor_callback, this, std::placeholders::_1));

        // Use a higher frequency timer for better control performance
        timer_ = this->create_wall_timer(50ms, std::bind(&AltitudeAttitudeControl::timer_callback, this));
        
        // Log successful initialization
        RCLCPP_INFO(this->get_logger(), "Fixed-Wing ASMC Controller initialized with parameters:");
        RCLCPP_INFO(this->get_logger(), "lambda=%.2f, phi=%.2f, alpha_0=%.3f, alpha_1=%.3f, k0_init=%.2f, k1_init=%.2f, k_max=%.2f",
           this->get_parameter("roll_lambda").as_double(),
           this->get_parameter("roll_phi").as_double(),
           this->get_parameter("roll_alpha_0").as_double(),
           this->get_parameter("roll_alpha_1").as_double(),
           this->get_parameter("roll_k0_init").as_double(),
           this->get_parameter("roll_k1_init").as_double(),
           this->get_parameter("roll_k_max").as_double());

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servos_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Safety flags
    bool attitude_received_ = false;
    bool position_received_ = false;
    bool gyro_received_ = false;
    
    // Timestamp tracking for accurate dt calculation
    uint64_t last_control_time_ = 0;

    int counter_;
    ASMC asmc_roll_;
    PID pid_pitch_;
    PID pid_yaw_;
    PID pid_altitude_;

    tf2Scalar desired_roll_ = 0.0f;
    tf2Scalar desired_pitch_ = 0.0f;
    tf2Scalar desired_yaw_ = M_PI / 2;

    float desired_altitude_ = 50.0f;
    float current_altitude_ = 0.0f;

    tf2Scalar current_roll_ = 0.0f;
    tf2Scalar current_pitch_ = 0.0f;
    tf2Scalar current_yaw_ = 0.0f;
    float current_roll_rate_ = 0.0f;
    
    // Method to load parameters from ROS param server
    void load_parameters() {
       // Create a new ASMC controller with the loaded parameters
        asmc_roll_ = ASMC(
            this->get_parameter("roll_lambda").as_double(),
            this->get_parameter("roll_phi").as_double(),
            this->get_parameter("roll_alpha_0").as_double(),
            this->get_parameter("roll_alpha_1").as_double(),
            this->get_parameter("roll_k0_init").as_double(),
            this->get_parameter("roll_k1_init").as_double(),
            this->get_parameter("roll_k_max").as_double()
        );
    }

    // New callback for sensor data to get direct gyro measurements
    void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
        // X-axis gyro reading corresponds to roll rate in body frame
        // For fixed-wing, may need to check sign depending on axis convention
        float raw_roll_rate = msg->gyro_rad[0];
        
        // Just store the raw value, filtering will happen in the control loop
        current_roll_rate_ = raw_roll_rate;
        gyro_received_ = true;
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        
        // Get current attitude
        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
        
        // Normalize yaw to [0, 2π] range
        if (current_yaw_ < 0) {
            current_yaw_ += 2.0 * M_PI;
        }
        
        attitude_received_ = true;
    }

    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        current_altitude_ = -msg->position[2];  // Negative in NED frame
        position_received_ = true;
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
        uint64_t current_time = now();
        
        // Calculate actual dt between control iterations (in seconds)
        float dt = (last_control_time_ > 0) ? 
                  (current_time - last_control_time_) / 1e6f : 0.05f;
        
        // Limit dt to reasonable bounds for numerical stability
        dt = std::clamp(dt, 0.001f, 0.1f);
        last_control_time_ = current_time;
        
        // Don't compute or send commands until we have all required data
        if (!attitude_received_ || !position_received_ || !gyro_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for complete sensor data before controlling");
            return;
        }
        
        // ASMC for roll control - using filtered gyro measurement
        float roll_cmd = asmc_roll_.compute(desired_roll_, current_roll_, current_roll_rate_, dt);
        
        // PID for other axes
        float yaw_error = desired_yaw_ - current_yaw_;
        // Normalize yaw error to [-π, π]
        if (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
        if (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;
        
        float yaw_cmd = std::clamp(pid_yaw_.compute(yaw_error, dt), -1.0f, 1.0f);
        
        float altitude_error = desired_altitude_ - current_altitude_;
        float throttle = std::clamp(pid_altitude_.compute(altitude_error, dt), 0.3f, 1.0f);
        
        // Pitch UP when below desired altitude
        float altitude_pitch_correction = std::clamp(0.15f * altitude_error, -0.2f, 0.2f);
        float target_pitch = desired_pitch_ + altitude_pitch_correction;
        float pitch_cmd = std::clamp(pid_pitch_.compute(target_pitch - current_pitch_, dt), -1.0f, 1.0f);
        
        auto now_us = now();
        
        // Publish thrust
        px4_msgs::msg::ActuatorMotors motors_msg{};
        motors_msg.timestamp = now_us;
        motors_msg.timestamp_sample = now_us;
        motors_msg.control[0] = throttle;
        motors_publisher_->publish(motors_msg);
        
        // Publish attitude commands - Note the mixing adjustments for fixed-wing
        px4_msgs::msg::ActuatorServos servos_msg{};
        servos_msg.timestamp = now_us;
        
        // Apply roll command to ailerons with differential mixing for better coordinated turns
        // More up on one side than down on the other can reduce adverse yaw
        servos_msg.control[0] = - roll_cmd;  // Left aileron
        servos_msg.control[1] =  roll_cmd;   // Right aileron
        
        servos_msg.control[2] = pitch_cmd;  // Elevator
        servos_msg.control[3] = yaw_cmd;    // Rudder
        servos_publisher_->publish(servos_msg);
        
        // Enhanced logging
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    "Roll: %.2f (des: %.2f) | Rate: %.3f | Rollcmd: %.3f | Sliding: %.3f | Alt: %.1f/%.1f | K0: %.3f | K1: %.3f",
        current_roll_, desired_roll_, current_roll_rate_, roll_cmd,
        asmc_roll_.get_sliding_surface(), current_altitude_, desired_altitude_, 
        asmc_roll_.get_adaptive_gain_k0(), asmc_roll_.get_adaptive_gain_k1());
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