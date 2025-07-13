#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

void messageCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("example_node"), "Received: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("example_node");
    auto publisher = node->create_publisher<std_msgs::msg::String>("example_topic", 10);
    auto subscriber = node->create_subscription<std_msgs::msg::String>(
        "example_topic", 10, messageCallback);
    
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok()) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        msg->data = "Hello, ROS 2!";
        publisher->publish(*msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
