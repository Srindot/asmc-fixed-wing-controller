#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class EulerAnglesPrinter : public rclcpp::Node
{
public:
    EulerAnglesPrinter() : Node("euler_angles_printer")
    {
        // QoS settings for PX4
        auto qos = rclcpp::QoS(10).best_effort().transient_local();

        // Subscribe to vehicle attitude topic
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                // Convert quaternion to Euler angles
                tf2::Quaternion q(
                    msg->q[1],  // x
                    msg->q[2],  // y
                    msg->q[3],  // z
                    msg->q[0]   // w (real part first in PX4, but TF2 expects w last)
                );
                
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                
                // Convert to degrees for easier reading
                double roll_deg = roll * 180.0 / M_PI;
                double pitch_deg = pitch * 180.0 / M_PI;
                double yaw_deg = yaw * 180.0 / M_PI;
                
                // Print both radians and degrees
                RCLCPP_INFO(this->get_logger(), 
                    "Euler Angles (rad): Roll: %.4f, Pitch: %.4f, Yaw: %.4f", 
                    roll, pitch, yaw);
                RCLCPP_INFO(this->get_logger(), 
                    "Euler Angles (deg): Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
                    roll_deg, pitch_deg, yaw_deg);
            });
            
        RCLCPP_INFO(this->get_logger(), "Euler Angles Printer node initialized.");
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EulerAnglesPrinter>());
    rclcpp::shutdown();
    return 0;
}
