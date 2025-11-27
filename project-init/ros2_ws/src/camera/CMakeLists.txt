cmake_minimum_required(VERSION 3.8)
project(camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(camera
    src/main.cpp
    src/CameraNode.cpp
)
ament_target_dependencies(camera rclcpp std_msgs interfaces)

install(TARGETS
    camera
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
