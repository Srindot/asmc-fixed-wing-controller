#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

class AltitudeDisplayNode : public rclcpp::Node {
public:
    AltitudeDisplayNode()
    : Node("altitude_display")
    {
        auto qos = rclcpp::QoS(10).best_effort();
        vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&AltitudeDisplayNode::vehicle_local_position_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        float altitude = -msg->z;  // Negative in NED (North-East-Down coordinate system)
        RCLCPP_INFO(this->get_logger(), "Current Altitude: %.2f meters", altitude);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AltitudeDisplayNode>());
    rclcpp::shutdown();
    return 0;
}
