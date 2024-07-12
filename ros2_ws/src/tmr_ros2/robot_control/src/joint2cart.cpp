#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "tm_kinematics/tm_kin.hpp" //need to put after Eigen

void read_config(std::string &j_name, std::string &jv_name, std::string &c_name, std::string &cv_name);
void read_txt(std::string path_name, std::vector<Eigen::Matrix<double, 6,1>> &data);
void write_txt(std::string path_name, std::vector<Eigen::Matrix<double, 6,1>> data);
void write_txt(std::string path_name, std::vector<Eigen::Matrix<double, 3,1>> data);

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string j_name, jv_name, c_name, cv_name;
    std::vector<Eigen::Matrix<double, 6,1>> joint_data, jointV_data, cartesianV_data;
    std::vector<Eigen::Matrix<double, 6,1>> cartesian_data;
    
    read_config(j_name, jv_name, c_name, cv_name);
    read_txt(j_name, joint_data);
    read_txt(jv_name, jointV_data);


    for(size_t index = 0; index < joint_data.size(); index++)
    {
        // cartesian data
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        std::array<double, 6> q;
        for(int i = 0; i < 6; i++)
            q[i] = joint_data[index][i];
        tm_kinematics::forward_tool(q, T, 0.2);
        Eigen::Matrix3d rotation = T.block<3, 3>(0, 0);
        Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0);
        //(0,3)->(3,3)
        auto car_postion = T.block<3, 1>(0, 3);
        Eigen::Matrix<double, 6,1> car;
        car << car_postion(0), car_postion(1), car_postion(2),
                euler_angles(2), euler_angles(1), euler_angles(0);
        cartesian_data.push_back(car);

        // cartesian velocity
        Eigen::Matrix<double, 6, 6> jacobian;
        jacobian = tm_jacobian::Forward_Jacobian_tool(joint_data[index], 0.2);
        // jacobian = tm_jacobian::Forward_Jacobian_d(joint_data[index]);
        auto car_v = jacobian*jointV_data[index];
        std::cout << car_v.transpose() << std::endl;
        cartesianV_data.push_back(car_v);
    }

    write_txt(cv_name, cartesianV_data);
    write_txt(c_name, cartesian_data);
    
  return 0;
}

void read_config(std::string &j_name, std::string &jv_name, std::string &c_name, std::string &cv_name)
{
    std::ifstream cFile("/home/frank/ros2_ws/joint2cart.txt");
    if (!cFile.is_open()) {
        std::cerr << "Couldn't open config file for reading.\n";
        return;
    }
    std::string line;
    while (std::getline(cFile, line)) {
        line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
        auto delimiterPos = line.find("=");
        if (line[0] == '#' || line.empty() || delimiterPos == std::string::npos)
            continue;
        auto name = line.substr(0, delimiterPos);
        auto value = line.substr(delimiterPos + 1);
        if (name == "joint_data") j_name = value;
        else if (name == "joint_velocity_data") jv_name = value;
        else if (name == "cartesian_data") c_name = value;
        else if (name == "cartesian_velocity_data") cv_name = value;
    }   
}

void read_txt(std::string path_name, std::vector<Eigen::Matrix<double, 6,1>> &data)
{
    std::ifstream inputFile;
    inputFile.open(path_name);
    if (!inputFile.is_open()) {
        std::cout << "Can't open file:" << path_name << std::endl;
        return;
    }

    std::string line;
    while (std::getline(inputFile, line)) 
    {
        Eigen::Matrix<double, 6,1> d;
        std::istringstream iss(line);
        double value;
        //read joint data(t, dof1.....dof6) from .txt
        for(size_t index = 0; index < 7; ++index){
            iss >> value;
            if(index == 0) continue;
            d[index-1] = value;
        } 
        data.push_back(d);
    } inputFile.close();
    std::cout << "Read data from File: " << path_name << std::endl;
}

void write_txt(std::string path_name, std::vector<Eigen::Matrix<double, 6,1>> data)
{
    size_t datalen = data.size();
    std::ofstream outputFile;
    outputFile.open(path_name);
    if (outputFile.is_open()){
        for (size_t index = 0; index < datalen; index++){
            for (size_t j = 0; j < 6; j++)
                outputFile << data[index][j] << " ";
            outputFile << std::endl;
        }
        outputFile.close();
        std::cout << "Output File saved: " << path_name << std::endl;
    }
    else
        std::cerr << "Unable to open file: " << path_name << std::endl;

}

void write_txt(std::string path_name, std::vector<Eigen::Matrix<double, 3,1>> data)
{
    size_t datalen = data.size();
    std::ofstream outputFile;
    outputFile.open(path_name);
    if (outputFile.is_open()){
        for (size_t index = 0; index < datalen; index++){
            for (size_t j = 0; j < 3; j++)
                outputFile << data[index][j] << " ";
            outputFile << std::endl;
        }
        outputFile.close();
        std::cout << "Output File saved: " << path_name << std::endl;
    }
    else
        std::cerr << "Unable to open file: " << path_name << std::endl;

}
