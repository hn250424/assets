#pragma once

#include "interfaces/constants.hpp"
#include "interfaces/msg/example.hpp"
#include "rclcpp/rclcpp.hpp"

class WorkstationNode : public rclcpp::Node {
public:
	WorkstationNode();

private:
	rclcpp::Subscription<interfaces::msg::Example>::SharedPtr sub_;

	void subscribe_sensor_state(const interfaces::msg::Example::SharedPtr msg);
};