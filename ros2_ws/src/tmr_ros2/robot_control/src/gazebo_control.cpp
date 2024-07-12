#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
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

#define NUMBER_OF_DOFS 6
using std::placeholders::_1;
using namespace std;
using namespace ruckig;
using namespace std::chrono_literals;
const float EPS = 1e-6;

double *T = new double[16];   
double *q_q  = new double[6];   
std::vector<double> VelPubCmd={0,0,0,0,0,0};

class gazebo_control : public rclcpp::Node{
  public:
    gazebo_control(const rclcpp::NodeOptions & options);
    ~gazebo_control(){}
    void robot_feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) ; 
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    double EndPointJoint[6];
    double EndPointCartesian[6];
    double RobotJointNow[6];
    double EndPointTTT[6];
    private: 
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joint_cmd_sub;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_robot;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_joint_cmd;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub;
    
    void publish_vel(std::vector<double> vel_cmd);
    bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ);
    bool CheckJointLimit(double *q);
    bool GetQfromInverseKinematics(double *CartesianPosition, double *q_inv);
    void RukigWayPointRun();
    void timer_callback();
    
};

gazebo_control::gazebo_control(const rclcpp::NodeOptions & options): rclcpp::Node("gazebo_control",options){
    callback_group_robot     = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    callback_group_joint_cmd = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions sub_robot_opt;
    sub_robot_opt.callback_group = callback_group_robot;
    rclcpp::SubscriptionOptions sub_cmd_opt  ;
    sub_cmd_opt.callback_group   = callback_group_joint_cmd;

    vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocity_controller/commands", 10);

    
    timer_ = this->create_wall_timer(100s, std::bind(&gazebo_control::timer_callback, this));

    robot_sub = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&gazebo_control::robot_feedback_callback, this, _1),sub_robot_opt);

    joint_cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "/joint_cmd", 10, std::bind(&gazebo_control::cmd_callback, this, _1),sub_cmd_opt);
    
}
void gazebo_control::robot_feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // printf("get\n");
    RobotJointNow[0] = msg->position[0];
    RobotJointNow[1] = msg->position[1];
    RobotJointNow[2] = msg->position[4];
    RobotJointNow[3] = msg->position[2];
    RobotJointNow[4] = msg->position[3];
    RobotJointNow[5] = msg->position[5];
    // printf("j0: %f\n", RobotJointNow[0]);
}
void gazebo_control::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        printf("cmd_callback\n");
        EndPointCartesian[0] = msg->linear.x;
        EndPointCartesian[1] = msg->linear.y;
        EndPointCartesian[2] = msg->linear.z;
        EndPointCartesian[3] = msg->angular.x;
        EndPointCartesian[4] = msg->angular.y;
        EndPointCartesian[5] = msg->angular.z;

        this->GetQfromInverseKinematics(EndPointCartesian, EndPointJoint);
        this->RukigWayPointRun();
        printf("ssss\n");

    }

void gazebo_control::RukigWayPointRun(){
// bool RukigWayPointRun(){
    printf("RukigWayPointRun\n");
    // PublisherNode publisher_node;
    // Create instances: the Ruckig OTG as well as input and output parameters
    const double control_cycle {0.025};
    const size_t max_number_of_waypoints {10};  // for memory allocation
    Ruckig<NUMBER_OF_DOFS> otg {control_cycle, max_number_of_waypoints};  // control cycle
    InputParameter<NUMBER_OF_DOFS> IP;
    OutputParameter<NUMBER_OF_DOFS> OP {max_number_of_waypoints};
    std::vector<double> VelocityCommand(6);
    printf("now : j0 %.3f j1 %.3f j2 %.3f j3 %.3f j4 %.3f j5 %.3f\n",
            RobotJointNow[0],RobotJointNow[1],RobotJointNow[2],RobotJointNow[3],RobotJointNow[4],RobotJointNow[5]);

    printf("goal : j0 %.3f j1 %.3f j2 %.3f j3 %.3f j4 %.3f j5 %.3f\n",
            EndPointJoint[0],EndPointJoint[1],EndPointJoint[2],EndPointJoint[3],EndPointJoint[4],EndPointJoint[5]);
    // Set input parameters
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP.current_position[i] = RobotJointNow[i];       
        IP.current_velocity[i] = 0;
        IP.current_acceleration[i] = 0;

        IP.target_position[i] = EndPointJoint[i];
        // IP.target_position[i] = EndPoint_ttt[i];
        IP.target_velocity[i] = 0;
        IP.target_acceleration[i] = 0;
        
        IP.max_acceleration[i] = 1;
        IP.max_jerk[i] = 3;
        IP.max_velocity[i] = 0.2;
    }

    IP.interrupt_calculation_duration = 500; // [µs]

    // Generate the trajectory within the control loop
    std::cout << "t | j1 | j2 | j3 | j4 | j5 | j6" << std::endl;
    while (otg.update(IP, OP) == Result::Working) 
    {
        // while (!vel_client->wait_for_service(1s)) {
        //     if (!rclcpp::ok()) {
        //         RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), 
        //             "Interrupted while waiting for the service. Exiting.");
        //         return false;
        //     }
        //     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), 
        //         "service not available, waiting again...");
        // }

        if (OP.new_calculation) {
            std::cout << "Updated the trajectory:" << std::endl;
            std::cout << "  Reached target position in " << OP.trajectory.get_duration() << " [s]." << std::endl;
            std::cout << "  Calculation in " << OP.calculation_duration << " [µs]." << std::endl;
        }

        // The area execution in 25ms real time sharp
        auto& p = OP.new_velocity;
        std::cout << OP.time << " " << p[0] << " " << p[1] << " " << p[2] 
            << " " << p[3]  << " " << p[4] << " " << p[5] << std::endl;
        VelocityCommand = { OP.new_velocity[0],
                            OP.new_velocity[1],
                            OP.new_velocity[2],
                            OP.new_velocity[3],
                            OP.new_velocity[4],
                            OP.new_velocity[5] };

        //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
        // Vrequest -> velocity = VelocityCommand;
        // auto Vresult = vel_client->async_send_request(Vrequest);

        

        this->publish_vel(VelocityCommand);
        // printf("aa\n");
        // Wait for the result.
        // if (rclcpp::spin_until_future_complete(node_setvel, Vresult) == rclcpp::executor::FutureReturnCode::SUCCESS)
        // {
        //     //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
        //     if( Vresult.get()-> ok )
        //         RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"OK");
        //     else
        //         RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"not OK");
        // }
        // else 
        //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");

        OP.pass_to_input(IP);
        usleep(25*1000);
    }

    // std::cout << "Trajectory duration: " << OP.trajectory.get_duration() << " [s]." << std::endl;
    // request_mode("off");
    std::cout << "ooo" << endl;

}
bool gazebo_control::pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ){
    Eigen::VectorXd sigma;  //vector of singular values
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

    sigma = svd.singularValues();
    int m = sigma.rows();

    for (int i = 0; i < m ; ++i)
    {
        if(sigma(i) > EPS)
            sigma(i) = 1.0/ sigma(i);
        else
            sigma(i) = 0.0;
    }

    invJ = svd.matrixV() * sigma.asDiagonal() * svd.matrixU().transpose();

    return true;
}
bool gazebo_control::CheckJointLimit(double *q){
    printf("CheckJointLimit: ");
    bool valid = true;
    if (abs(q[0]) > 265 * DEG2RAD){
        RCLCPP_WARN(get_logger(),"[Position] 1st joint position out of limit (270) : %lf", q[0] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[1]) > 175 * DEG2RAD){
        RCLCPP_WARN(get_logger(),"[Position] 2nd joint position out of limit (180): %lf", q[1] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[2]) > 150 * DEG2RAD){
        RCLCPP_WARN(get_logger(),"[Position] 3rd joint position out of limit (155): %lf", q[2] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[3]) > 175 * DEG2RAD) {
        RCLCPP_WARN(get_logger(),"[Position] 4th joint position out of limit (180): %lf", q[3] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[4]) > 175 * DEG2RAD){
        RCLCPP_WARN(get_logger(),"[Position] 5th joint position out of limit (180): %lf", q[4] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[5]) > 265 * DEG2RAD){
        RCLCPP_WARN(get_logger(),"[Position] 6th joint position out of limit (180): %lf", q[5] * RAD2DEG);
        valid = false;
    }
    else
        valid = true;
    printf("%s\n",valid?"true":"false");
    return valid;
}
bool gazebo_control::GetQfromInverseKinematics(double *CartesianPosition, double *q_inv){
    printf("GetQfromInverseKinematics\n");

    Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
    Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
    Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
    Eigen::Matrix<float, 4, 4> T_;

    T_ << 0., 0., 0., CartesianPosition[0],
        0., 0., 0., CartesianPosition[1],
        0., 0., 0., CartesianPosition[2],
        0., 0., 0., 1.;

    T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);

    tm_jacobian::Matrix2DoubleArray(T_, T);
    tm_kinematics::inverse(T, q_q);
    for(int i=0;i<6;i++)
        q_inv[i] = q_q[i];


    delete[] T;
    return this->CheckJointLimit(q_inv);
}
void gazebo_control::publish_vel(std::vector<double> vel_cmd){
    printf("publish_vel\n");
    auto message = std_msgs::msg::Float64MultiArray();
    std::vector<double> aaa={};
    VelPubCmd = vel_cmd;
    message.data = VelPubCmd;
    // auto curr_thread = string_thread_id();
    //     // Prep display message
    // RCLCPP_INFO(this->get_logger(), "\n<< %s>>",curr_thread.c_str());
    this->vel_pub->publish(message);
  }

void gazebo_control::timer_callback() {
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = VelPubCmd;
    this->vel_pub->publish(message);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gazebo_control>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),8,true);
    executor.add_node(node);
    printf("threads   %ld\n",executor.get_number_of_threads());
    executor.spin();
    

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
