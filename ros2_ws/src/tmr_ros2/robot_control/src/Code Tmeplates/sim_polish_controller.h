#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <queue>
#include <string>

#include <Eigen/Core>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include <ruckig/ruckig.hpp> //need to put after Eigen
#include "tm_kinematics/tm_kin.hpp" //need to put after Eigen


#define NUM_DOF 6

using std::placeholders::_1;
using namespace std::chrono_literals;
using Array = std::array<double, NUM_DOF>;
using PathList = std::vector<Array>;
const float EPS = 1e-6;
const float Ideal_Velocity = 0.01;

typedef struct{
    std::unique_ptr<PathList> J_ptr;
    std::unique_ptr<PathList> Jv_ptr;
    std::vector<float> t;
    Array J_first;
    Array J_ready;
    size_t Len;
    size_t Index;
}WayPointData;

typedef enum{
    FREE, ONE, 
    WPSTART, WP2READY, WPWORK, WPWAITNEXT, WPDONE
} TaskMode;

typedef struct{
    std::vector<double> time;
    PathList j;
    PathList jv;
} ExperementData;
 
namespace polish_control_interface {
    class robot_control : public rclcpp::Node{
        public:
            robot_control(const rclcpp::NodeOptions & options, std::string fileName, std::string outputfile)
            : rclcpp::Node("robot_control", options), TxtFileName(fileName), TxtOutPutFileName(outputfile)
            {
                vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/joint_velocity_controller/commands", 10);
                robot_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/joint_states", 10, std::bind(&robot_control::joint_callback, this, _1));
                joint_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/joint_cmd", 10, std::bind(&robot_control::cmd_callback, this, _1));
                polish_sub = this->create_subscription<std_msgs::msg::Bool>(
                    "/polish_cmd", 10, std::bind(&robot_control::mode_callback, this, _1));
                timer_ = this->create_wall_timer(40ms, std::bind(&robot_control::ruckig_state_manage, this));

                ruckig_input.max_velocity.fill(0.6);
                ruckig_input.max_acceleration.fill(100);
                ruckig_input.max_jerk.fill(100.0);
                current_joint.fill(0.0);
                current_joint_vel.fill(0.0);
                current_joint_acl.fill(0.0);
                std::fill(&EndPointJ[0], &EndPointJ[0] + 6, 0.0);
                std::fill(&EndPointC[0], &EndPointC[0] + 6, 0.0);
                ControlMode = FREE;
            }

        private:
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_sub;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joint_cmd_sub;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr polish_sub;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub;
            rclcpp::TimerBase::SharedPtr timer_;
            //ruckig data
            ruckig::InputParameter<NUM_DOF> ruckig_input;
            ruckig::OutputParameter<NUM_DOF> ruckig_output;
            std::unique_ptr<ruckig::Ruckig<NUM_DOF>> ruckig_ptr;
            Array current_joint, current_joint_vel, current_joint_acl, EndPointJ;
            double EndPointC[6];
            WayPointData WP_data;
            TaskMode ControlMode;
            bool PolishMode = false;
            std::string TxtFileName, TxtOutPutFileName;

            void ruckig_state_manage();
            void ruckig_move();
            //waypoint move
            bool WayPointDataSet();
            void NextWayPointRuckig(); 
            //One point move
            void NextPointRuckig(); 
            void initializeRuckigState();
            //sub&pub
            void publish_vel(std::vector<double> vel_cmd);
            void joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_msg); 
            void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void mode_callback(const std_msgs::msg::Bool::SharedPtr bool_msg);   
            //for experiment
            void OutPutEx2F();

            std::vector<Eigen::VectorXd> Car_path;  
            std::vector<Eigen::VectorXd> carti_p;
            std::vector<double> normmm;
            rclcpp::Time start_time;
            rclcpp::Time end_time;
            ExperementData exdata;
    };
}

namespace kinematic_tool {
    bool CheckJointLimit(Array q);
    bool GetQfromInverseKinematics(double *CartesianPosition, Array &q_inv, Array startJ);
    bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ);
    bool GetQdfromLinearJacobian(Array curQ, Array EFF_Velocity, Array& qd);
}



