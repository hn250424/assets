#include "camera/CameraNode.hpp"

using namespace std::chrono_literals;

CameraNode::CameraNode()
    : Node("camera_node"), frame_count_(0), loop_rate_(1s)
{
    pub_ = this->create_publisher<std_msgs::msg::String>("camera/image", 10);

    RCLCPP_INFO(this->get_logger(), "Camera node started");
}

void CameraNode::start()
{
    // Use a wall timer to periodically publish image frames.
    // This approach is preferred over calling spin_some in a loop,
    // because spin_some only processes callbacks while it is running, 
    // which may cause delays or missed updates for periodic tasks.
    // Wall timers integrate better with ROS 2's executor, avoid busy-waiting,
    // and ensure callbacks are called at consistent intervals.
    timer_ = this->create_wall_timer(
        loop_rate_.period(),
        std::bind(&CameraNode::publish_image_frame, this));
}

void CameraNode::publish_image_frame()
{
    auto msg = std_msgs::msg::String();
    msg.data = "Image Frame_" + std::to_string(frame_count_++);

    RCLCPP_INFO(this->get_logger(), "Capture and Publishing Image: '%s'", msg.data.c_str());
    pub_->publish(msg);
}
