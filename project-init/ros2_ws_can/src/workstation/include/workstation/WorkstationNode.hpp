#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class WorkstationNode : public rclcpp::Node {
public:
    WorkstationNode();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    void subscribe_sensor_state(const std_msgs::msg::String::SharedPtr msg);
};