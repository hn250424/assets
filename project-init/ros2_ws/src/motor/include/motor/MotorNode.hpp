#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/move_motor.hpp"

class MotorNode : public rclcpp::Node {
public:
    MotorNode();
private:
    void handleMotorRequest(
        const std::shared_ptr<interfaces::srv::MoveMotor::Request> request,
        std::shared_ptr<interfaces::srv::MoveMotor::Response> response
    );

    rclcpp::Service<interfaces::srv::MoveMotor>::SharedPtr srv_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;
    
};

#endif