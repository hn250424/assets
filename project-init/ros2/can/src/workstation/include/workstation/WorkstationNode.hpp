#pragma once

#include <chrono>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/example.hpp"
#include "interfaces/constants.hpp"

class WorkstationNode : public rclcpp::Node {
public:
    WorkstationNode();

private:
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Service<interfaces::srv::Example>::SharedPtr srv_;
    std::mt19937 rng_;
    std::uniform_int_distribution<int> dist_;

    // void subscribe_sensor_state(const std_msgs::msg::String::SharedPtr msg);
    void ack(
        const std::shared_ptr<interfaces::srv::Example::Request> request,
        std::shared_ptr<interfaces::srv::Example::Response> response
    );
};