cmake_minimum_required(VERSION 3.8)
project(motor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(motor 
    src/main.cpp
    src/MotorNode.cpp
)
ament_target_dependencies(motor rclcpp std_msgs interfaces)

install(TARGETS
    motor
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
