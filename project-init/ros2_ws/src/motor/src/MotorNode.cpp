#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor/MotorNode.hpp"

using namespace std;

MotorNode::MotorNode() : 
    Node("motor_node"),
    gen_(std::random_device{}()),
    dist_(0, 10)
{
    srv_ = this->create_service<interfaces::srv::MoveMotor>(
        "move_motor_service",
        std::bind(&MotorNode::handleMotorRequest, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    RCLCPP_INFO(this->get_logger(), "Motor node service started");
}

void MotorNode::handleMotorRequest(
    const std::shared_ptr<interfaces::srv::MoveMotor::Request> request,
    std::shared_ptr<interfaces::srv::MoveMotor::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received request for image_no: %s", request->image_no.c_str());

    response->success = true;
    response->move_amount = dist_(gen_);

    RCLCPP_INFO(this->get_logger(), "Responding with success=true, move_amount=%d", response->move_amount);
}