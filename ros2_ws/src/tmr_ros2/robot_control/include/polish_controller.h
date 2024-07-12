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
#include "tm_msgs/msg/feedback_state.hpp"
#include "tm_msgs/srv/set_velocity.hpp"
#include "tm_msgs/srv/set_positions.hpp"
#include "tm_msgs/srv/send_script.hpp"

#include <ruckig/ruckig.hpp> //need to put after Eigen
#include "tm_kinematics/tm_kin.hpp" //need to put after Eigen


#define NUM_DOF 6

using std::placeholders::_1;
using namespace std::chrono_literals;
using Array = std::array<double, NUM_DOF>;
using PathList = std::vector<Array>;
const float EPS = 1e-6;
const float Ideal_Velocity = 0.008;

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
    class robot_control : public rclcpp::Node {
        public:
            robot_control(const rclcpp::NodeOptions & options, std::string fileName, std::string outputfile)
            : rclcpp::Node("robot_control", options), TxtFileName(fileName), TxtOutPutFileName(outputfile)
            {
                p_client = this->create_client<tm_msgs::srv::SetPositions>("set_positions");
                vel_client = this->create_client<tm_msgs::srv::SetVelocity>("set_velocity");
                client_sendscript = this->create_client<tm_msgs::srv::SendScript>("send_script");
                // TM-robot 6-Axis state
                robot_sub = this->create_subscription<tm_msgs::msg::FeedbackState>(
                    "feedback_states", 10, std::bind(&robot_control::joint_callback, this, _1));
                cartesian_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/cartesian_cmd", 10, std::bind(&robot_control::cartesian_cmd_callback, this, _1));

                joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/joint_cmd", 10, std::bind(&robot_control::joint_cmd_callback, this, _1));
                polish_sub = this->create_subscription<std_msgs::msg::Bool>(
                    "/polish_cmd", 10, std::bind(&robot_control::mode_callback, this, _1));
                timer_ = this->create_wall_timer(40ms, std::bind(&robot_control::ruckig_state_manage, this));

                ruckig_input.max_velocity.fill(0.3);
                ruckig_input.max_acceleration.fill(1.5);
                ruckig_input.max_jerk.fill(100);
                current_joint.fill(0.0);
                current_joint_vel.fill(0.0);
                current_joint_acl.fill(0.0);
                std::fill(&EndPointJ[0], &EndPointJ[0] + 6, 0.0);
                std::fill(&EndPointC[0], &EndPointC[0] + 6, 0.0);
                ControlMode = FREE;
            }

        private:
            rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr robot_sub;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cartesian_cmd_sub;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;
            //system to polish mode which is waypoint control
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr polish_sub;
            //velocity control
            rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client_sendscript;
            rclcpp::Client<tm_msgs::srv::SetVelocity>::SharedPtr vel_client;
            //position control
            rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr p_client;
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
            //txt file name/path
            std::string TxtFileName, TxtOutPutFileName;
            //for experiment
            rclcpp::Time start_time;
            rclcpp::Time end_time;
            ExperementData exdata;

            void ruckig_state_manage();
            void ruckig_move();
            bool request_mode(std::string RT_MODE);
            //waypoint move
            bool WayPointDataSet();
            void NextWayPointRuckig(); 
            //One point move
            void NextPointRuckig(); 
            void initializeRuckigState();
            void PositionRun(std::vector<double> targetJ);
            //sub&pub
            void joint_callback(const tm_msgs::msg::FeedbackState::SharedPtr joint_msg); 
            void cartesian_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void mode_callback(const std_msgs::msg::Bool::SharedPtr bool_msg);  
            void joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg); 
            //for experiment
            void OutPutEx2F(); 
            
    };
}

namespace kinematic_tool {
    bool CheckJointLimit(Array q);
    bool GetQfromInverseKinematics(double *CartesianPosition, Array &q_inv, Array startJ);
    bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ);
    bool GetQdfromLinearJacobian(Array curQ, Array EFF_Velocity, Array& qd);
}



