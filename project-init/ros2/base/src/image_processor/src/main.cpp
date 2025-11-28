#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_processor/ImageProcessorNode.hpp"

using namespace std;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<ImageProcessorNode>());
    rclcpp::shutdown();
    return 0;
}