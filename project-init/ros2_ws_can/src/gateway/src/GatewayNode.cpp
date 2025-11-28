#include "gateway/GatewayNode.hpp"

using namespace std::chrono_literals;

GatewayNode::GatewayNode() : Node("GatewayNode") {
    RCLCPP_INFO(this->get_logger(), "GatewayNode started");

    pub_ = this->create_publisher<std_msgs::msg::String>("/sensor/state", 10);

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

    timer_ = this->create_wall_timer(100ms, [this]() {
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

            auto ros_msg = std_msgs::msg::String();
            ros_msg.data = msg_str;
            pub_->publish(ros_msg);
            RCLCPP_INFO(this->get_logger(), "Gateway publish: '%s'", ros_msg.data.c_str());
        }
    });
}

GatewayNode::~GatewayNode() {
    if (sock_ >= 0) {
        close(sock_);
    }
}
