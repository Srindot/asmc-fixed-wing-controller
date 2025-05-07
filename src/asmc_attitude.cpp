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

// Simple PID controller used for yaw and altitude.
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

// Utility function: saturated sign (boundary layer)
// This returns sgn(s) if |s| > epsilon and s/epsilon otherwise.
float sat_sign(float s, float epsilon) {
    if (std::fabs(s) > epsilon)
        return std::copysign(1.0f, s);
    else
        return s / epsilon;
}

// ASMC controller that follows classical adaptive SMC formulation.
// Here we use the formulation:
//   e = q - q_d  
//   s = dot(q) - Lambda*(q - q_d)
//   u = -(A + K*sat_sign(s, epsilon))*s
//   
//   With K defined as:
//   ρ(t) = K̂0(t) + K̂1(t)|q(t)|,
//   
//   And adaptation laws:
//   ˙K̂0(t) = |s(t)| − α0 K̂0(t),
//   ˙K̂1(t) = |s(t)||q(t)| − α1 K̂1(t),
struct ASMC_Slide {
    float lambda;      // sliding surface factor
    float A;           // fixed gain (non-adaptive)
    float k0_nominal;  // nominal (minimum) value for k0
    float k1_nominal;  // nominal (minimum) value for k1
    float epsilon;     // boundary layer threshold
    float alpha0;      // adaptation rate for k0
    float alpha1;      // adaptation rate for k1
    
    // Adapted gain components (initialized at nominal values)
    float k0;          // adaptive component for constant term
    float k1;          // adaptive component for |q| term

    ASMC_Slide(float lambda_, float A_, float k0_nominal_, float k1_nominal_, 
               float alpha0_, float alpha1_, float epsilon_ = 0.01f)
        : lambda(lambda_), A(A_), k0_nominal(k0_nominal_), k1_nominal(k1_nominal_),
          alpha0(alpha0_), alpha1(alpha1_), epsilon(epsilon_),
          k0(k0_nominal_), k1(k1_nominal_) {}
    
    // Reset adaptation state
    void reset() {
        k0 = k0_nominal;
        k1 = k1_nominal;
    }
    
    // Get the current total K gain for the sliding mode term
    // Now uses the classical form ρ(t) = K̂0(t) + K̂1(t)|q(t)|
    float get_K(float q) const {
        return k0 + k1 * std::fabs(q);
    }
    
    // Update adaptive gains based on sliding surface value
    // Implementing the classical adaptation laws
    void update_gains(float s, float q, float dt) {
        // Update k0 based on the adaptation law ˙K̂0(t) = |s(t)| − α0 K̂0(t)
        k0 += (std::fabs(s) - alpha0 * k0) * dt;
        
        // Update k1 based on the adaptation law ˙K̂1(t) = |s(t)||q(t)| − α1 K̂1(t)
        k1 += (std::fabs(s) * std::fabs(q) - alpha1 * k1) * dt;
        
        // Ensure gains don't go below nominal values
        k0 = std::max(k0, k0_nominal);
        k1 = std::max(k1, k1_nominal);
    }
};

class AltitudeAttitudeControl : public rclcpp::Node {
public:
    AltitudeAttitudeControl()
        : Node("altitude_attitude_control"), counter_(0),
          // Keep the working PID for yaw and altitude.
          pid_yaw_(0.1f, 0.2f, 0.01f),
          pid_altitude_(0.1f, 0.5f, 0.6f),
          // ASMC controllers with adaptation parameters
          // Parameters: lambda, A, k0_nominal, k1_nominal, alpha0, alpha1, epsilon
          asmc_roll_(10.0f, 0.01f, 0.2f, 0.1f, 0.05f, 0.02f, 0.02f),
          asmc_pitch_(2.22f, 0.027f, 0.0f, 0.0f, 0.3f, 0.1f, 0.3f),
          last_time_(0)
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
    // Publishers and subscribers.
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servos_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    PID pid_yaw_;
    PID pid_altitude_;
    ASMC_Slide asmc_roll_;
    ASMC_Slide asmc_pitch_;
    // Desired states.
    tf2Scalar desired_roll_ = 0.0f;
    tf2Scalar desired_pitch_ = 0.0f;
    tf2Scalar desired_yaw_ = 3 * M_PI / 2;
    float desired_altitude_ = 50.0f;
    // Current state.
    tf2Scalar current_roll_ = 0.0f;
    tf2Scalar current_pitch_ = 0.0f;
    tf2Scalar current_yaw_ = 0.0f;
    float current_altitude_ = 0.0f;
    // To compute time difference.
    uint64_t last_time_;
    // For computing angular rates.
    tf2Scalar prev_roll_ = 0.0f;
    tf2Scalar prev_pitch_ = 0.0f;

    // Callback to update attitude angles.
    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        // The quaternion order for PX4 is [w, x, y, z] but our tf2::Quaternion
        // is constructed as (x, y, z, w).
        tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
        if (current_yaw_ < 0)
            current_yaw_ += 2.0f * M_PI;
    }

    // Callback to update altitude.
    void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        // In NED coordinate frame.
        current_altitude_ = -msg->position[2];
    }

    // Timer callback for periodic control updates.
    void timer_callback() {
        publish_offboard_control_mode();
        if (counter_ == 10)
            engage_offboard_mode();
        if (counter_ == 15)
            arm();
        // Compute dt in seconds.
        uint64_t current_time = now();
        float raw_dt = (last_time_ > 0) ? static_cast<float>(current_time - last_time_) / 1e6f : 0.1f;
        last_time_ = current_time;
        // (Optional) scale dt if you want faster adaptation:
        const float dt_scale = 1.0f; // change this if you wish to “speed‐up” adaptation.
        float dt = raw_dt * dt_scale;
        publish_actuator_values(dt);
        counter_++;
    }

    // Publish offboard mode.
    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now();
        msg.direct_actuator = true;
        offboard_control_mode_publisher_->publish(msg);
    }

    // Control computation for roll and pitch using the new ASMC_Slide formulation.
    void publish_actuator_values(float dt) {
        // -------- Roll channel (e.g., aileron) -----------
        float error_roll = current_roll_ - desired_roll_;
        float roll_rate = (current_roll_ - prev_roll_) / dt;
        float s_roll = roll_rate - asmc_roll_.lambda * error_roll;

        // Update adaptive gains with both sliding surface and error values
        asmc_roll_.update_gains(s_roll, error_roll, dt);

        // Get current adapted gains based on error (not sliding surface)
        float K_roll = asmc_roll_.get_K(error_roll);

        // Compute control with fixed A and adaptive K
        float roll_cmd = (asmc_roll_.A * s_roll + 0*K_roll * sat_sign(s_roll, asmc_roll_.epsilon));
        prev_roll_ = current_roll_;

        // -------- Pitch channel (e.g., elevator) -----------
        float error_pitch = current_pitch_ - desired_pitch_;
        float pitch_rate = (current_pitch_ - prev_pitch_) / dt;
        float s_pitch = pitch_rate - asmc_pitch_.lambda * error_pitch;

        // Update adaptive gains with both sliding surface and error values
        asmc_pitch_.update_gains(s_pitch, error_pitch, dt);

        // Get current adapted gains based on error (not sliding surface)
        float K_pitch = asmc_pitch_.get_K(error_pitch);

        // Compute control with fixed A and adaptive K
        float pitch_cmd = (asmc_pitch_.A * s_pitch + 0.01*K_pitch * sat_sign(s_pitch, asmc_pitch_.epsilon));
        prev_pitch_ = current_pitch_;

        // For yaw and altitude we continue to use PID control.
        float yaw_cmd = std::clamp(pid_yaw_.compute(desired_yaw_ - current_yaw_, dt), -1.0f, 1.0f);
        float altitude_error = desired_altitude_ - current_altitude_;
        float throttle = std::clamp(pid_altitude_.compute(altitude_error, dt), 0.3f, 1.0f);

        auto now_us = now();

        // Publish motor command (throttle).
        px4_msgs::msg::ActuatorMotors motors_msg{};
        motors_msg.timestamp = now_us;
        motors_msg.timestamp_sample = now_us;
        motors_msg.control[0] = throttle;
        motors_publisher_->publish(motors_msg);

        // Publish servo commands:
        // Differential aileron: left and right for roll,
        // Elevator for pitch, and rudder for yaw.
        px4_msgs::msg::ActuatorServos servos_msg{};
        servos_msg.timestamp = now_us;
        servos_msg.control[0] = -roll_cmd;  // left aileron
        servos_msg.control[1] = roll_cmd;   // right aileron
        servos_msg.control[2] = pitch_cmd;  // elevator
        servos_msg.control[3] = 0.0f;    // rudder
        servos_publisher_->publish(servos_msg);

        // Enhanced logging to show adaptive gains
        RCLCPP_INFO(this->get_logger(),
                    "Roll: err=%.2f, s=%.2f, k0=%.2f, k1=%.2f, K=%.2f, cmd=%.2f | "
                    "Pitch: err=%.2f, s=%.2f, k0=%.2f, k1=%.2f, K=%.2f, cmd=%.2f",
                    error_roll, s_roll, asmc_roll_.k0, asmc_roll_.k1, K_roll, roll_cmd,
                    error_pitch, s_pitch, asmc_pitch_.k0, asmc_pitch_.k1, K_pitch, pitch_cmd);
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
