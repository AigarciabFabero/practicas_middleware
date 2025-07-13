#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <thread>

class SimpleSubscriber : public rclcpp::Node {
public:
    SimpleSubscriber() : Node("simple_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,  // Ajusta la profundidad de la cola aquí
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
                std::this_thread::sleep_for(std::chrono::seconds(1));  // Simula un procesamiento lento
            });
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}
