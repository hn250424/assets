#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("camera_node");
    auto pub = node->create_publisher<std_msgs::msg::String>("camera/image", 10);
    rclcpp::WallRate loop_rate(1s);

    int frame_count = 0;

    RCLCPP_INFO(node->get_logger(), "Camera node started");

    while (rclcpp::ok()) {
        auto msg = std_msgs::msg::String();
        msg.data = "Image Frame_" + to_string(frame_count++);
        
        RCLCPP_INFO(node->get_logger(), "Capture and Publishing Image: '%s'", msg.data.c_str());
        pub->publish(msg);
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}