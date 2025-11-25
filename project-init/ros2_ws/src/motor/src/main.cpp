#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "motor/MotorNode.hpp"

using namespace std;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}