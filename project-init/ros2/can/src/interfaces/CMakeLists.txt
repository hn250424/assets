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

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})

ament_package()