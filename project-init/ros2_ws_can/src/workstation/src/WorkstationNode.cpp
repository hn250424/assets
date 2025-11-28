#include "workstation/WorkstationNode.hpp"

WorkstationNode::WorkstationNode() : Node("WorkstationNode") {
    RCLCPP_INFO(this->get_logger(), "Workstation started");

    sub_ = this->create_subscription<std_msgs::msg::String>(
        "/sensor/state", 
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->subscribe_sensor_state(msg);
        }
    );
}

void WorkstationNode::subscribe_sensor_state(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received sensor state: '%s'", msg->data.c_str());
}
