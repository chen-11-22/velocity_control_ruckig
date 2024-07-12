#include "tm_kinematics/tm_kin.hpp" //need to put after Eigen

#define NUM_DOF 6
#define DANGEROUS_ZONE 0.3
#define JOINTLIMIT_SPD_123 150
// #define JOINTLIMIT_SPD_456 200
#define JOINTLIMIT_SPD_456 150


using Array = std::array<double, NUM_DOF>;
using PathList = std::vector<Array>;
const float EPS = 1e-6;

bool CheckJointLimit(Array q);
bool GetQfromInverseKinematics(double *CartesianPosition, Array &q_inv, Array startJ);
bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ);
bool GetQdfromLinearJacobian(Array curQ, Array EFF_Velocity, Array& qd);