#include "workstation/WorkstationNode.hpp"

WorkstationNode::WorkstationNode() : Node(interfaces::nodes::WORKSTATION) {
    RCLCPP_INFO(this->get_logger(), "Workstation started");

    // sub_ = this->create_subscription<std_msgs::msg::String>(
    //     "/sensor/state", 
    //     10,
    //     [this](const std_msgs::msg::String::SharedPtr msg) {
    //         this->subscribe_sensor_state(msg);
    //     }
    // );

    srv_ = this->create_service<interfaces::srv::Example>(
        interfaces::services::UPDATE_SENSOR,
        std::bind(&WorkstationNode::ack, this, std::placeholders::_1, std::placeholders::_2)
    );
    rng_.seed(std::random_device{}());
    dist_ = std::uniform_int_distribution<int>(0, 1);
}

// void WorkstationNode::subscribe_sensor_state(const std_msgs::msg::String::SharedPtr msg) {
//     RCLCPP_INFO(this->get_logger(), "[Workstation] subscribe: '%s'", msg->data.c_str());
// }

void WorkstationNode::ack(
    const std::shared_ptr<interfaces::srv::Example::Request> request,
    std::shared_ptr<interfaces::srv::Example::Response> response
) {
    RCLCPP_INFO(this->get_logger(), "[Workstation] request: %s", request->sensor_value.c_str());

    response->result = dist_(rng_);

    RCLCPP_INFO(this->get_logger(), "[Workstation] response: %d", response->result);

}