#include "sensor/SensorNode.hpp"
#include "can_driver/CanDriver.hpp"

using namespace std::chrono_literals;

SensorNode::SensorNode() : Node(interfaces::nodes::SENSOR) {
	printf("[Sensor] SensorNode started\n");

	pub_ = this->create_publisher<interfaces::msg::Example>(
		interfaces::topics::UPDATE_SENSOR,
		rclcpp::QoS(1)
			.reliable()
			.transient_local()
			.keep_last(1));

	auto can = std::make_shared<CanDriver>();
	if (!can->open("vcan0")) {
		printf("[Sensor] Failed to open CAN interface\n");
		// return;
	}

	// Create a dedicated worker thread for CAN communication.
	// This mimics an 'Event Listener'.
	can_thread_ = std::thread([this, can]() {
		can_frame frame{};

		while (rclcpp::ok() && running_) {
			if (can->read(frame)) {
				auto ros_msg = interfaces::msg::Example();

				const uint32_t can_id = frame.can_id & CAN_EFF_MASK;
				int id_0 = (can_id >> 24) & 0xFF;
				int id_1 = (can_id >> 16) & 0xFF;
				int id_2 = (can_id >> 8) & 0xFF;
				int id_3 = (can_id >> 0) & 0xFF;

				const uint8_t data = frame.data;
				ros_msg.acting = data[0];
				ros_msg.pos = data[1];
				ros_msg.detecting[1] = data[2] & 0x01;
				ros_msg.detecting[0] = (data[2] >> 1) & 0x01;
				ros_msg.detecting_len = data[3];

				printf("[Sensor] publish sensor data acting: %d / pos: %d / detecting: ", ros_msg.acting, ros_msg.pos);
				for (int i = 0; i < ros_msg.detecting_len; ++i) {
					printf("%d ", ros_msg.detecting[i]);
				}
				printf("\n");

				pub_->publish(ros_msg);
			}
		}
	});
}

SensorNode::~SensorNode() {
	running_ = false;

	// joinable() checks if the thread is active.
	// join() waits until the thread finishes its loop and exits.
	// This ensures the node doesn't destroy itself while the thread is still running.
	if (can_thread_.joinable()) {
		can_thread_.join();
	}
}