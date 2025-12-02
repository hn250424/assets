cmake_minimum_required(VERSION 3.8)
project(interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files "msg/Example.msg")
set(srv_files "srv/Example.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
)

ament_package()