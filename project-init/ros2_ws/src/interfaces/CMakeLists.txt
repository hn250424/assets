cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
    "srv/MoveMotor.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${srv_files}
)

ament_package()