#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>

typedef struct{
    std::string record;
    char type; //joint or cart
    std::vector<double> axes;
}txtData;

class interface : public rclcpp::Node //letting the class member could use rclcpp's function
{
    public: 
        //constructor
        interface(const rclcpp::NodeOptions & options, std::string fileName, std::string outputfile)
        : rclcpp::Node("TM5_900_Interface", options), TxtFileName(fileName), TxtOutPutFileName(outputfile)
        //rclcpp::Node initialize
        {
            cart_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>( //create pub
                "/cart_cmd", 10);
            joint_velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>( //create pub
                "/joint_cmd", 10);
        }
        //mode
        void control();
    
    private:
        void CPUB(float x,float y,float z,float rx,float ry,float rz);
        void JPUB(float x,float y,float z,float rx,float ry,float rz);
        //function
        void InitialPose();
        void P2P();
        void readData();
        void printData();
        void TXT();

        //publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cart_velocity_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joint_velocity_publisher;
        
        //txt
        txtData txt_data;
        std::vector<txtData> txt_data_list;
        std::ifstream input_file;
        std::string TxtFileName, TxtOutPutFileName;
};

void interface::control()
{
    char Mode;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << " Welcone to TM5_900 sim control interface." << std::endl;
    std::cout << " Dear user, please pressing:              " << std::endl;
    std::cout << " I to Initial Pose                        " << std::endl;
    std::cout << " P for Point to Point control             " << std::endl;
    std::cout << " T for txt P2P controlling multiple points" << std::endl;
    std::cout << " Q to quit                                " << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Your type is: ";
    std::cin  >> Mode;
    std::cout << "------------------------------------------" << std::endl;
    switch (Mode)
    {
        case 'i':
        case 'I':
            {
                InitialPose();
            }
            break;
        case 'p':
        case 'P':
            {
                P2P();
            }
            break;
        case 't':
        case 'T':
            {
                TXT();
            }
            break;
        case 'q':
        case 'Q':
            {
                std::cout <<"Thanks for using, good bye."<< std::endl;
                rclcpp::shutdown();
            }
            break;
        default:
            {
                std::cout << "Please type again:" << std::endl;
            }
            break;
    }
}

void interface::CPUB(float x,float y,float z,float rx,float ry,float rz)
{
    geometry_msgs::msg::Twist message;
    message.linear.x = x;
    message.linear.y = y;
    message.linear.z = z; 
    message.angular.x = rx;
    message.angular.y = ry;
    message.angular.z = rz;
    this-> cart_velocity_publisher ->publish(message);
    std::cout << "Moving to Cartesian Space Position: ";
    std::cout << message.linear.x << "," << message.linear.y << "," << message.linear.z << ","
              << message.angular.x << "," << message.angular.y << "," << message.angular.z;
    std::cout <<" ...\n";
}

void interface::JPUB(float x,float y,float z,float rx,float ry,float rz)
{
    geometry_msgs::msg::Twist message;
    message.linear.x = x;
    message.linear.y = y;
    message.linear.z = z; 
    message.angular.x = rx;
    message.angular.y = ry;
    message.angular.z = rz;
    this-> joint_velocity_publisher ->publish(message);
    std::cout << "Moving to Joint Space Position: ";
    std::cout << message.linear.x << "," << message.linear.y << "," << message.linear.z << ","
              << message.angular.x << "," << message.angular.y << "," << message.angular.z;
    std::cout <<" ...\n";      
}

void interface::InitialPose()
{
    geometry_msgs::msg::Twist message;
    std::cout << "Moving back to Inital Position..." << std::endl;
    message.linear.x = 0;
    message.linear.y = 0;
    message.linear.z = 0;
    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = 0;
    this-> joint_velocity_publisher ->publish(message);
}

void interface::P2P()
{
    geometry_msgs::msg::Twist message;

    std::cout << "Type the Position:" << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    std::cin >> message.linear.x >> message.linear.y >> message.linear.z
             >> message.angular.x >> message.angular.y >> message.angular.z;

    char space;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Type C for Cartesian Space, J for Joint Space: " << std::endl;
    std::cin >> space;
    std::cout << "------------------------------------------" << std::endl;
    if(space == 'c' || space == 'C')
    {
        this-> cart_velocity_publisher ->publish(message);
        std::cout << "Moving to Cartesian Space Position: ";
        std::cout << message.linear.x << "," << message.linear.y << "," << message.linear.z << ","
                  << message.angular.x << "," << message.angular.y << "," << message.angular.z;
        std::cout <<" ...\n";
        std::cout << "Process Ended." << std::endl;
    }
    else if (space == 'j' || space == 'J')
    {
        this-> joint_velocity_publisher ->publish(message);
        std::cout << "Moving to Joint Space Position: ";
        std::cout << message.linear.x << "," << message.linear.y << "," << message.linear.z << ","
                  << message.angular.x << "," << message.angular.y << "," << message.angular.z;
        std::cout <<" ...\n";
        std::cout << "Process Ended." << std::endl;
    }
    else
    {
        std::cout << "Please type again" << std::endl;
    }
}

void interface::readData()
{
    input_file.open(TxtFileName);
    if(!input_file){
        std::cerr << "Failed to open the file:" << TxtFileName << std::endl;
    }

    std::string record;
    char type;
    double value;
    while (input_file >> record >> type) 
    {
        txt_data.axes.clear();
        txt_data.record = record;
        txt_data.type = type;
            
        for (int i = 0; i < 6; ++i) {
            if (input_file >> value) {
                txt_data.axes.push_back(value);
            } 
            else {
                std::cerr << "Failed to read axes data." << std::endl;
            }
        }
        txt_data_list.push_back(txt_data);
    }
    input_file.close();
}

void interface::printData()
{
     for (const auto& txt_data : txt_data_list) {
        std::cout << "Data: " << txt_data.record << ", Type: " << txt_data.type << ", Axes: ";
        for (double axis : txt_data.axes) {
            std::cout << axis << " ";
        }
        std::cout << std::endl;
    }
}

void interface::TXT()
{   
    //initialize txt_data_list;
    txt_data_list.clear();

    std::cout << "Please type the txt filename:" << std::endl;
    std::cin >> TxtFileName;
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Start Reading" << TxtFileName << "..." << std::endl;
    std::cout << "------------------------------------------" << std::endl;
    interface::readData();
    interface::printData();
    std::cout << "------------------------------------------" << std::endl;

    rclcpp::Rate loop_rate(1); //execution frequency

    for(const auto& txt_data : txt_data_list)
        {
            if(txt_data.type == 'c' || txt_data.type == 'C')
            {
                CPUB(txt_data.axes[0],txt_data.axes[1],txt_data.axes[2],
                txt_data.axes[3],txt_data.axes[4],txt_data.axes[5]);
                loop_rate.sleep();
            }
            else if(txt_data.type == 'j' || txt_data.type == 'J')
            {
                JPUB(txt_data.axes[0],txt_data.axes[1],txt_data.axes[2],
                txt_data.axes[3],txt_data.axes[4],txt_data.axes[5]);
                loop_rate.sleep();
            }
        }
        std::cout << "------------------------------------------" << std::endl;
        std::cout << "Process Ended." << std::endl;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string fileName, expfile_name;

    auto node = std::make_shared<interface>(rclcpp::NodeOptions(), fileName, expfile_name);

    while(rclcpp::ok())
    {
        node -> control();
    }
    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}