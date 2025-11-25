#ifndef IMAGE_PROCESSOR_NODE_HPP
#define IMAGE_PROCESSOR_NODE_HPP

#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/move_motor.hpp"

class ImageProcessorNode : public rclcpp::Node {
public:
    ImageProcessorNode();
private:
    void imageCallback(const std_msgs::msg::String::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Client<interfaces::srv::MoveMotor>::SharedPtr client_;
    std::mt19937 gen_;
    std::uniform_int_distribution<int> dist_;
};

#endif