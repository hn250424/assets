cmake_minimum_required(VERSION 3.8)
project(image_processor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(image_processor 
    src/main.cpp
    src/ImageProcessorNode.cpp
)
ament_target_dependencies(image_processor rclcpp std_msgs interfaces)

install(TARGETS
    image_processor
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
