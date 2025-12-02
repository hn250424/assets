#include "gateway/GatewayNode.hpp"

using namespace std::chrono_literals;

const int MAX_RETRY_COUNT = 5;
const int TIMER_DELAY = 3;

GatewayNode::GatewayNode() : Node("GatewayNode") {
    RCLCPP_INFO(this->get_logger(), "GatewayNode started");

    // pub_ = this->create_publisher<std_msgs::msg::String>("/sensor/state", 10);
    client_ = this->create_client<interfaces::srv::Example>("/workstation/sensor");

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
        return;
    }

    ifreq ifr{};
    std::strcpy(ifr.ifr_name, "vcan0");
    ioctl(sock_, SIOCGIFINDEX, &ifr);

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int socket_bind_ret = bind(sock_, (struct sockaddr*)&addr, sizeof(addr));
    if (socket_bind_ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket");
        close(sock_);
        return;
    }

    fcntl(sock_, F_SETFL, O_NONBLOCK);

    can_timer_ = this->create_wall_timer(100ms, [this]() {
        can_frame frame{};
        int nbytes = read(sock_, &frame, sizeof(frame));

        if (nbytes > 0) {
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
            // RCLCPP_INFO(this->get_logger(), "[Gateway] publish: '%s'", ros_msg.data.c_str());
        }
    });
}

GatewayNode::~GatewayNode() {
    if (sock_ >= 0) {
        close(sock_);
    }
}

void GatewayNode::update_sensor_value(int retry_count) {
    auto handled = std::make_shared<std::atomic<bool>>(false);

    // if (!client_->wait_for_service(std::chrono::seconds(1))) {
    //     RCLCPP_WARN(this->get_logger(), "[Gateway] Service not available");
    //     if (retry_count < 5) {
    //         schedule_retry(retry_count);
    //     } else {
    //         finish_update(false);
    //     }
    //     return;
    // }

    auto req = std::make_shared<interfaces::srv::Example::Request>();
    req->sensor_value = last_sensor_value_;
    RCLCPP_INFO(this->get_logger(), "[Gateway] request: '%s'", last_sensor_value_.c_str());

    if (timeout_timer_) {
        timeout_timer_->cancel();
    }

    timeout_timer_ = this->create_wall_timer(
        std::chrono::seconds(TIMER_DELAY),
        [this, retry_count, handled]() {
            bool expected = false;
            if (handled->compare_exchange_strong(expected, true)) {
                timeout_timer_->cancel();
                RCLCPP_ERROR(this->get_logger(), "[Gateway] Response timeout!");
                
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
                RCLCPP_INFO(this->get_logger(), "[Gateway] response: '%d'", res->result);
    
                if (res->result == 0 && retry_count < MAX_RETRY_COUNT) {
                    schedule_retry(retry_count);
                } else {
                    finish_update(res->result == 1);
                }
            }
        }
    );
}

void GatewayNode::schedule_retry(int retry_count) {
    if (retry_timer_) {
        retry_timer_->cancel();
    }

    auto t_ = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    *t_ = this->create_wall_timer(
        std::chrono::seconds(TIMER_DELAY),
        [this, retry_count, t_]() {
            (*t_)->cancel();
            RCLCPP_WARN(this->get_logger(), "[Gateway] retrying... (%d)", retry_count + 1);
            update_sensor_value(retry_count + 1);
        }
    );

    retry_timer_ = *t_;
}

void GatewayNode::finish_update(bool success) {
    if (retry_timer_) {
        retry_timer_->cancel();
        retry_timer_.reset();
    }

    if (success) {
        RCLCPP_INFO(this->get_logger(), "[Gateway] ✓ Successfully updated!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "[Gateway] ✗ Failed or max retries reached");
    }

    is_updating_ = false;
}