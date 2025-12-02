#pragma once

#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/example.hpp"

class GatewayNode : public rclcpp::Node {
public:
    GatewayNode();
    ~GatewayNode();

private:
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Client<interfaces::srv::Example>::SharedPtr client_;

    int sock_;
    
    rclcpp::TimerBase::SharedPtr can_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    std::atomic<bool> is_updating_{false};
    std::string last_sensor_value_;

    void update_sensor_value(int retry_count = 0);
    void schedule_retry(int retry_count);
    void finish_update(bool success);
};