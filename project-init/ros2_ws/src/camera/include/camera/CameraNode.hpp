#pragma once

#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/action/generator.hpp"

using Generator = interfaces::action::Generator;
using GoalHandleGenerator = rclcpp_action::ClientGoalHandle<Generator>;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    void start();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    int frame_count_;
    rclcpp::WallRate loop_rate_;

    rclcpp_action::Client<Generator>::SharedPtr action_client_;
    GoalHandleGenerator::SharedPtr current_goal_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool startGenerator_;

    void publish_image_frame();
    void send_weight_measurement();
};