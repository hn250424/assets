cmake_minimum_required(VERSION 3.8)
project(gateway)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(gateway
    src/main.cpp
    src/GatewayNode.cpp
)
ament_target_dependencies(gateway rclcpp std_msgs interfaces)

install(TARGETS
    gateway
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
