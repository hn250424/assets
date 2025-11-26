cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

set(srv_files
    "srv/MoveMotor.srv"
)

set(action_files
    "action/Generator.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_files}
    ${action_files}
    DEPENDENCIES unique_identifier_msgs
)

ament_package()