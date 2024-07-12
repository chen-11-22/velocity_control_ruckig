#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
// #include <Eigen/Eigen>
// #include <Eigen/Dense>
#include <Eigen/Core>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <ruckig/ruckig.hpp> //need to put after Eigen
#include "tm_kinematics/tm_kin.hpp" //need to put after Eigen


#define NUM_DOF 6

using std::placeholders::_1;
using namespace std::chrono_literals;
 
 
namespace control_interface {
    class gazebo_control : public rclcpp::Node{
        public:
            gazebo_control(const rclcpp::NodeOptions & options)
            : rclcpp::Node("gazebo_control", options)
            {
                vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/joint_velocity_controller/commands", 10);
                robot_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/joint_states", 10, std::bind(&gazebo_control::joint_callback, this, _1));
                joint_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/joint_cmd", 10, std::bind(&gazebo_control::cmd_callback, this, _1));
                timer_ = this->create_wall_timer(40ms, std::bind(&gazebo_control::ruckig_state_manage, this));
                max_velocities.fill(0.08);
                max_accelerations.fill(1.0);
                max_jerks.fill(3.0);
                current_joint.fill(0.0);
                current_joint_vel.fill(0.0);
                current_joint_acl.fill(0.0);
                std::fill(&EndPointJ[0], &EndPointJ[0] + 6, 0.0);
                std::fill(&EndPointC[0], &EndPointC[0] + 6, 0.0);
            }

        private:
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_sub;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joint_cmd_sub;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_robot;
            rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_joint_cmd;
            ruckig::InputParameter<NUM_DOF> ruckig_input;
            ruckig::OutputParameter<NUM_DOF> ruckig_output;
            std::unique_ptr<ruckig::Ruckig<NUM_DOF>> ruckig_ptr;
            std::array<double, NUM_DOF> max_velocities, max_accelerations, max_jerks;
            std::array<double, NUM_DOF> current_joint, current_joint_vel, current_joint_acl;
            double EndPointJ[6];
            double EndPointC[6];
            bool ruckig_activate = false;

            void ruckig_state_manage();
            void publish_vel(std::vector<double> vel_cmd);
            void getNextRuckigInput();
            void initializeRuckigState();
            void joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_msg); 
            void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            bool GetQfromInverseKinematics(double *CartesianPosition, double *q_inv);
    };
}
bool CheckJointLimit(double *q);



/*ros2 multi-thread*/
/*
rclcpp::SubscriptionOptions sub_robot_opt;
rclcpp::SubscriptionOptions sub_cmd_opt;
callback_group_robot
    this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
callback_group_joint_cmd
    this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
sub_cmd_opt.callback_group = callback_group_joint_cmd;
sub_robot_opt.callback_group = callback_group_robot;  
robot_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&gazebo_control::joint_callback, this, _1), sub_robot_opt);
joint_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "/joint_cmd", 10, std::bind(&gazebo_control::cmd_callback, this, _1), sub_cmd_opt);
*/
///