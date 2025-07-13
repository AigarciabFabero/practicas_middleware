#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread> // Para std::this_thread::get_id()

// Callback para topic1
void callback1(const std_msgs::msg::String::SharedPtr msg) {
    auto thread_id = std::this_thread::get_id();
    RCLCPP_INFO(
        rclcpp::get_logger("callback1"),
        "Received from topic1: '%s' on thread [%ld]",
        msg->data.c_str(),
        std::hash<std::thread::id>{}(thread_id) // Convierte el ID de hilo en un valor numérico
    );
}

// Callback para topic2
void callback2(const std_msgs::msg::String::SharedPtr msg) {
    auto thread_id = std::this_thread::get_id();
    RCLCPP_INFO(
        rclcpp::get_logger("callback2"),
        "Received from topic2: '%s' on thread [%ld]",
        msg->data.c_str(),
        std::hash<std::thread::id>{}(thread_id)
    );
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Crear un nodo compartido
    auto node = rclcpp::Node::make_shared("multi_threaded_node");

    // Crear suscripciones para los topics
    auto sub1 = node->create_subscription<std_msgs::msg::String>(
        "topic1", 10, callback1);
    auto sub2 = node->create_subscription<std_msgs::msg::String>(
        "topic2", 10, callback2);

    // Crear publicadores para los topics
    auto pub1 = node->create_publisher<std_msgs::msg::String>("topic1", 10);
    auto pub2 = node->create_publisher<std_msgs::msg::String>("topic2", 10);

    // Timers para publicar periódicamente en los topics
    auto timer1 = node->create_wall_timer(
        std::chrono::seconds(1), [pub1]() {
            auto thread_id = std::this_thread::get_id();
            auto message = std_msgs::msg::String();
            message.data = "Message from topic1";
            pub1->publish(message);
            RCLCPP_INFO(
                rclcpp::get_logger("publisher1"),
                "Published to topic1: '%s' on thread [%ld]",
                message.data.c_str(),
                std::hash<std::thread::id>{}(thread_id)
            );
        });

    auto timer2 = node->create_wall_timer(
        std::chrono::seconds(2), [pub2]() {
            auto thread_id = std::this_thread::get_id();
            auto message = std_msgs::msg::String();
            message.data = "Message from topic2";
            pub2->publish(message);
            RCLCPP_INFO(
                rclcpp::get_logger("publisher2"),
                "Published to topic2: '%s' on thread [%ld]",
                message.data.c_str(),
                std::hash<std::thread::id>{}(thread_id)
            );
        });

    // Crear un MultiThreadedExecutor con un número específico de hilos
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Ejecutar el executor con múltiples hilos
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
