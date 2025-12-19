#include "workstation/WorkstationNode.hpp"

WorkstationNode::WorkstationNode() : Node(interfaces::nodes::WORKSTATION) {
	printf("[Workstation] WorkstationNode started\n");

	sub_ = this->create_subscription<interfaces::msg::Example>(
		interfaces::topics::UPDATE_SENSOR,
		rclcpp::QoS(1)
			.reliable()
			.transient_local()
			.keep_last(1),
	    [this](const interfaces::msg::Example::SharedPtr msg) {
		    this->subscribe_sensor_state(msg);
	    });
}

void WorkstationNode::subscribe_sensor_state(const interfaces::msg::Example::SharedPtr msg) {
	printf("[Workstation] subscribe: acting: %d / pos: %d / detecting: ", msg->acting, msg->pos);
	for (int i = 0; i < msg->detecting_len; ++i) {
		printf("%d ", msg->detecting[i]);
	}
	printf("\n");
}
