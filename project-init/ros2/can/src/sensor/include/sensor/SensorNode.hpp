#pragma once

#include "interfaces/constants.hpp"
#include "interfaces/msg/example.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <thread>

class SensorNode : public rclcpp::Node {
public:
	SensorNode();
	~SensorNode();

private:
	rclcpp::Publisher<interfaces::msg::Example>::SharedPtr pub_;

	std::thread can_thread_;
	std::atomic<bool> running_{true};
};