#include "rclcpp/rclcpp.hpp"
#include "gateway/GatewayNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto gateway_node = std::make_shared<GatewayNode>();
    rclcpp::spin(gateway_node);

    rclcpp::shutdown();
    return 0;
}