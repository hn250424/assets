#pragma once

#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GatewayNode : public rclcpp::Node {
public:
    GatewayNode();
    ~GatewayNode();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int sock_;

    // void publish_sensor_state(std_msgs::msg::String msg);
};