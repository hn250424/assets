cmake_minimum_required(VERSION 3.8)
project(sensor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(can_driver REQUIRED)

include_directories(include)

add_executable(sensor
    src/main.cpp
    src/SensorNode.cpp
)

ament_target_dependencies(sensor rclcpp std_msgs interfaces can_driver)

install(TARGETS
    sensor
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
