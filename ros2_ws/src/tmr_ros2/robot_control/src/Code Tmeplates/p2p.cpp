#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publish_data_node");

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher = 
    node->create_publisher<geometry_msgs::msg::Twist>("/joint_cmd", 10);
    
    while(rclcpp::ok())
    {      
        geometry_msgs::msg::Twist message;
        std::cout << "Type the velocity" << std::endl;
        std::cin >> message.linear.x >> message.linear.y >> message.linear.z
                 >> message.angular.x >> message.angular.y >> message.angular.z;

        publisher->publish(message);
        RCLCPP_INFO(node->get_logger(), 
            "Axes: %.3f %.3f %.3f %.3f %.3f %.3f",
            message.linear.x, message.linear.y, message.linear.z, 
            message.angular.x , message.angular.y, message.angular.z);  
        
        RCLCPP_INFO(node->get_logger(),"Process Ended");
    }
    return EXIT_SUCCESS;
    rclcpp::shutdown();
}