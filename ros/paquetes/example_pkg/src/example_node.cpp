#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

void messageCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_node");
    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<std_msgs::String>("example_topic", 10);
    ros::Subscriber subscriber = nh.subscribe("example_topic", 10, messageCallback);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello, ROS!";
        publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
