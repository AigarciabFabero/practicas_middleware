cmake_minimum_required(VERSION 3.5)
project(example_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(example_node src/example_node.cpp)
ament_target_dependencies(example_node rclcpp std_msgs)

install(TARGETS example_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
