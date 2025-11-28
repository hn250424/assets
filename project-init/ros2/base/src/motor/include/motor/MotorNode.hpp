#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/move_motor.hpp"
#include "interfaces/action/generator.hpp"

using Generator = interfaces::action::Generator;
using GoalHandleGenerator = rclcpp_action::ClientGoalHandle<Generator>;

class MotorNode : public rclcpp::Node {
public:
    MotorNode();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<interfaces::srv::MoveMotor>::SharedPtr srv_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;

    rclcpp_action::Client<Generator>::SharedPtr action_client_;
    GoalHandleGenerator::SharedPtr current_goal_handle_;

    void handleMotorRequest(
        const std::shared_ptr<interfaces::srv::MoveMotor::Request> request,
        std::shared_ptr<interfaces::srv::MoveMotor::Response> response
    );

    void send_weight_measurement();
    
};

#endif