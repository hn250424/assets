#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    void start();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    int frame_count_;
    rclcpp::WallRate loop_rate_;

    rclcpp::TimerBase::SharedPtr timer_;
    
    void publish_image_frame();
};