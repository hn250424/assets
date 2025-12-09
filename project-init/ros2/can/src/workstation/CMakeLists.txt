cmake_minimum_required(VERSION 3.8)
project(workstation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/WorkstationNode.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs interfaces)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
