cmake_minimum_required(VERSION 3.8)
project(workstation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(workstation
    src/main.cpp
    src/WorkstationNode.cpp
)
ament_target_dependencies(workstation rclcpp std_msgs interfaces)

install(TARGETS
    workstation
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
