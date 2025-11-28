#include "rclcpp/rclcpp.hpp"
#include "workstation/WorkstationNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto workstation_node = std::make_shared<WorkstationNode>();
    rclcpp::spin(workstation_node);

    rclcpp::shutdown();
    return 0;
}