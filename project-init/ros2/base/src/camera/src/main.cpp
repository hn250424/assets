#include "rclcpp/rclcpp.hpp"
#include "camera/CameraNode.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto camera_node = std::make_shared<CameraNode>();
    camera_node->start();

    rclcpp::spin(camera_node);

    rclcpp::shutdown();
    return 0;
}