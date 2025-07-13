#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher() : Node("simple_publisher"), count_(0) { // Inicializa el contador
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SimplePublisher::publish_message, this));
    }

private:
    void publish_message() {
        auto message = std_msgs::msg::String();
        message.data = "Hola ROS2 from C++: " + std::to_string(count_); // Añade el contador al mensaje
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
        publisher_->publish(message);
        count_++; // Incrementa el contador
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_; // Atributo para el contador
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
