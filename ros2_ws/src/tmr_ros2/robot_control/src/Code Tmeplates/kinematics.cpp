#include <vector>
#include <array>
#include <string>
#include <algorithm>
#include <Eigen/Dense>
#include <ruckig/ruckig.hpp>
#include "tm_kinematics/tm_kin.hpp"



using Eigen::MatrixXd;
using Eigen::VectorXd;

 

bool CheckJointLimit(double *q)
{
    bool valid = true;

    if (abs(q[0]) > 265 * DEG2RAD)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[Position] 1st joint position out of limit (270) : %lf", q[0] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[1]) > 175 * DEG2RAD)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[Position] 2nd joint position out of limit (180): %lf", q[1] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[2]) > 150 * DEG2RAD)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[Position] 3rd joint position out of limit (155): %lf", q[2] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[3]) > 175 * DEG2RAD) 
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[Position] 4th joint position out of limit (180): %lf", q[3] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[4]) > 175 * DEG2RAD)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[Position] 5th joint position out of limit (180): %lf", q[4] * RAD2DEG);
        valid = false;
    }
    else if (abs(q[5]) > 265 * DEG2RAD)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"[Position] 6th joint position out of limit (180): %lf", q[5] * RAD2DEG);
        valid = false;
    }
    else
        valid = true;

    return valid;
}

double *T = new double[16];   
double *q_q  = new double[6];
bool GetQfromInverseKinematics(double *CartesianPosition, double *q_inv)
{
    // for(int i = 0; i < 6; i++)
    //     std::cout << CartesianPosition[i] <<std::endl;
    printf("GetQfromInverseKinematics\n");
    Eigen::Matrix<float, 4, 4> T_;
    Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
    Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
    Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
    // double *T = new double[16]; 

    T_ << 0., 0., 0., CartesianPosition[0],
        0., 0., 0., CartesianPosition[1],
        0., 0., 0., CartesianPosition[2],
        0., 0., 0., 1.;
    T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);

    tm_jacobian::Matrix2DoubleArray(T_, T);
    // int num_sol = tm_kinematics::inverse(T, q_q);
    tm_kinematics::inverse(T, q_q);
    for(int i=0;i<6;i++)
        q_inv[i] = q_q[i];
    delete[] T;
    return CheckJointLimit(q_inv);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    double EndPointC[10] = {0};   
    double EndPointJ[10] = {0}; 
    bool within_limit = true;
    EndPointC[0] = -0.17;
    EndPointC[1] = -0.2;
    EndPointC[2] = 1;
    EndPointC[3] = 1.57;
    EndPointC[4] = 0;
    EndPointC[5] = 0;
    within_limit = GetQfromInverseKinematics(EndPointC, EndPointJ);
    for(int i = 0; i < 6; i++)
        std::cout << EndPointJ[i] <<std::endl;
        
  return 0;
}