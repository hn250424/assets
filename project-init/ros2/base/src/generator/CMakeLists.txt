cmake_minimum_required(VERSION 3.8)
project(generator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(generator 
    src/main.cpp
)
ament_target_dependencies(generator rclcpp rclcpp_action std_msgs interfaces)

install(TARGETS
    generator
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
