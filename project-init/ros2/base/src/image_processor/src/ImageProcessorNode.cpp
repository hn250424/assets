#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_processor/ImageProcessorNode.hpp"
#include "interfaces/srv/move_motor.hpp"

using namespace std;

ImageProcessorNode::ImageProcessorNode() : 
    Node("image_processor_node"),
    gen_(random_device{}()),
    dist_(1, 2)
{
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "camera/image", 
        10,
        bind(&ImageProcessorNode::imageCallback, this, placeholders::_1)
    );

    client_ = this->create_client<interfaces::srv::MoveMotor>("move_motor_service");
    
    RCLCPP_INFO(this->get_logger(), "Image processor node started");
}

void ImageProcessorNode::imageCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received Image: %s", msg->data.c_str());

    int flag = dist_(gen_);
    RCLCPP_INFO(this->get_logger(), "Flag generated: %d", flag);

    if (flag == 2) {
        RCLCPP_INFO(this->get_logger(), "Prepare request...");
        auto request = std::make_shared<interfaces::srv::MoveMotor::Request>();
        request->image_no = msg->data;

        RCLCPP_INFO(this->get_logger(), "Sending request: image_no=%s", request->image_no.c_str());

        auto result_future = 
            client_->async_send_request(
                request, 
                [this, msg](rclcpp::Client<interfaces::srv::MoveMotor>::SharedFuture future) {
                    auto response = future.get();
                    RCLCPP_INFO(
                        this->get_logger(), 
                        "Service response received for image: %s, success=%d, move_amount=%d",
                        msg->data.c_str(), 
                        response->success,
                        response->move_amount
                    );
                }
            );
    }
}