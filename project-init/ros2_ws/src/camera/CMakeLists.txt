cmake_minimum_required(VERSION 3.8)
project(camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(camera src/main.cpp)
ament_target_dependencies(camera rclcpp std_msgs)

install(TARGETS
    camera
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
