#pragma once

#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "interfaces/srv/example.hpp"

class SensorNode : public rclcpp::Node {
public:
    SensorNode();
    ~SensorNode();

private:
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Client<interfaces::srv::Example>::SharedPtr client_;

    rclcpp::TimerBase::SharedPtr can_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    std::atomic<bool> is_updating_{false};
    std::string last_sensor_value_;

    void update_sensor_value(int retry_count = 0);
    void schedule_retry(int retry_count);
    void finish_update(bool success);
};