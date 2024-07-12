#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

class DataReader {
public:
    struct Data {
        std::string record;
        char type; //joint or cart
        std::vector<double> axes;
    };

    std::vector<Data> data_list;

    DataReader(const std::string& filename) : input_file(filename) {
        if (!input_file) {
            std::cerr << "Failed to open the file:" << filename<< std::endl;
            return;
        }
    }

    void readData() {
        std::string record;
        char type;
        double value;
        
        while (input_file >> record >> type) {
            Data data;
            data.record = record;
            data.type = type;
            
            for (int i = 0; i < 6; ++i) {
                if (input_file >> value) {
                    data.axes.push_back(value);
                } else {
                    std::cerr << "Failed to read axes data." << std::endl;
                }
            }
            
            data_list.push_back(data);
        }
    }

    void printData() const {
        for (const auto& data : data_list) {
            std::cout << "Data: " << data.record << ", Type: " << data.type << ", Axes: ";
            for (double axis : data.axes) {
                std::cout << axis << " ";
            }
            std::cout << std::endl;
        }
    }

private:
    std::ifstream input_file;
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("publish_data_node");

    DataReader fileReader("input.txt");
    fileReader.readData();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher = 
    node->create_publisher<geometry_msgs::msg::Twist>("/joint_cmd", 10);
    
    while(rclcpp::ok()){      
        for (const auto& data : fileReader.data_list) {
            rclcpp::Rate loop_rate(3);
            geometry_msgs::msg::Twist message;

            if(data.type == 'j') //joint space data
            {
                message.linear.x = data.axes[0];
                message.linear.y = data.axes[1];
                message.linear.z = data.axes[2];
                message.angular.x = data.axes[3];
                message.angular.y = data.axes[4];
                message.angular.z = data.axes[5];
            }
            else if(data.type == 'c') //cartesian space data
            {
                std::cout<<"It should be transform to Joint Space Data!"<<std::endl;
                message.linear.x = data.axes[0];
                message.linear.y = data.axes[1];
                message.linear.z = data.axes[2];
                message.angular.x = data.axes[3];
                message.angular.y = data.axes[4];
                message.angular.z = data.axes[5];
            }
            publisher->publish(message);

            RCLCPP_INFO(node->get_logger(), 
            "Published Data: %s, Type: %c, Axes: %.3f %.3f %.3f %.3f %.3f %.3f",
            data.record.c_str(), data.type, 
            data.axes[0], data.axes[1], data.axes[2], 
            data.axes[3], data.axes[4], data.axes[5]);  
            std::this_thread::sleep_for(std::chrono::seconds(1));
            rclcpp::spin_some(node);
        }
        RCLCPP_INFO(node->get_logger(),"Process Ended");
        rclcpp::shutdown();
    }
    return EXIT_SUCCESS;
}