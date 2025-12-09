#include "sensor/SensorNode.hpp"
#include "can_driver/CanDriver.hpp"

using namespace std::chrono_literals;

constexpr int MAX_RETRY_COUNT = 5;
constexpr int TIMER_DELAY = 3;

SensorNode::SensorNode() : Node(interfaces::nodes::SENSOR) {
    RCLCPP_INFO(this->get_logger(), "SensorNode started");

    // pub_ = this->create_publisher<std_msgs::msg::String>("/sensor/state", 10);
    client_ = this->create_client<interfaces::srv::Example>(interfaces::services::UPDATE_SENSOR);
    
    auto can = std::make_shared<CanDriver>();
    if (!can->open("vcan0")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CAN interface");
        // return;
    }

    can_timer_ = this->create_wall_timer(100ms, [this, can]() {
        can_frame frame{};
        
        if (can->read(frame)) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "ID: 0x%X, Data:", frame.can_id);
            std::string msg_str(buf);

            for (int i = 0; i < frame.can_dlc; ++i) {
                char byte_buf[4];
                std::snprintf(byte_buf, sizeof(byte_buf), " %02X", frame.data[i]);
                msg_str += byte_buf;
            }

            last_sensor_value_ = msg_str;
            bool expected = false;
            if (is_updating_.compare_exchange_strong(expected, true)) {
                update_sensor_value();
            }

            // auto ros_msg = std_msgs::msg::String();
            // ros_msg.data = msg_str;
            // pub_->publish(ros_msg);
            // RCLCPP_INFO(this->get_logger(), "[Sensor] publish: '%s'", ros_msg.data.c_str());
        }
    });
}

SensorNode::~SensorNode() {

}

void SensorNode::update_sensor_value(int retry_count) {
    auto handled = std::make_shared<std::atomic<bool>>(false);

    // if (!client_->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_WARN(this->get_logger(), "[Sensor] Service not available");
    //     if (retry_count < 5) {
    //         schedule_retry(retry_count);
    //     } else {
    //         finish_update(false);
    //     }
    //     return;
    // }

    auto req = std::make_shared<interfaces::srv::Example::Request>();
    req->sensor_value = last_sensor_value_;
    RCLCPP_INFO(this->get_logger(), "[Sensor] request: '%s'", last_sensor_value_.c_str());

    if (timeout_timer_) {
        timeout_timer_->cancel();
    }

    timeout_timer_ = this->create_wall_timer(
        std::chrono::seconds(TIMER_DELAY),
        [this, retry_count, handled]() {
            bool expected = false;
            if (handled->compare_exchange_strong(expected, true)) {
                timeout_timer_->cancel();
                RCLCPP_ERROR(this->get_logger(), "[Sensor] Response timeout!");
                
                if (retry_count < MAX_RETRY_COUNT) {
                    schedule_retry(retry_count);
                } else {
                    finish_update(false);
                }
            }
        }
    );

    client_->async_send_request(
        req,
        [this, retry_count, handled](rclcpp::Client<interfaces::srv::Example>::SharedFuture future) {
            bool expected = false;
            if (handled->compare_exchange_strong(expected, true)) {
                timeout_timer_->cancel();
    
                auto res = future.get();
                RCLCPP_INFO(this->get_logger(), "[Sensor] response: '%d'", res->result);
    
                if (res->result == 0 && retry_count < MAX_RETRY_COUNT) {
                    schedule_retry(retry_count);
                } else {
                    finish_update(res->result == 1);
                }
            }
        }
    );
}

void SensorNode::schedule_retry(int retry_count) {
    if (retry_timer_) {
        retry_timer_->cancel();
    }

    auto t_ = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    *t_ = this->create_wall_timer(
        std::chrono::seconds(TIMER_DELAY),
        [this, retry_count, t_]() {
            (*t_)->cancel();
            RCLCPP_WARN(this->get_logger(), "[Sensor] retrying... (%d)", retry_count + 1);
            update_sensor_value(retry_count + 1);
        }
    );

    retry_timer_ = *t_;
}

void SensorNode::finish_update(bool success) {
    if (retry_timer_) {
        retry_timer_->cancel();
        retry_timer_.reset();
    }

    if (success) {
        RCLCPP_INFO(this->get_logger(), "[Sensor] ✓ Successfully updated!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "[Sensor] ✗ Failed or max retries reached");
    }

    is_updating_ = false;
}