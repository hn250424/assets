#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "motor/MotorNode.hpp"

using namespace std;

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);

//     auto motor_node = std::make_shared<MotorNode>();
//     motor_node->start();

//     rclcpp::spin(motor_node);
    
//     rclcpp::shutdown();
//     return 0;
// }


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}