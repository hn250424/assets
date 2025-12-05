#include "rclcpp/rclcpp.hpp"
#include "sensor/SensorNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto sensor_node = std::make_shared<SensorNode>();
    rclcpp::spin(sensor_node);

    rclcpp::shutdown();
    return 0;
}