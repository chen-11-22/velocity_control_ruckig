#include "waypoint_planner.h"

bool pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ)
{
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


bool CheckJointLimit(Array q)
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

bool GetQfromInverseKinematics(double *CartesianPosition, Array &q_inv, Array startJ)
{
	Eigen::Matrix<float, 4, 4> T_;
	Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
	Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
    T_ << 0., 0., 0., CartesianPosition[0],
		0., 0., 0., CartesianPosition[1],
		0., 0., 0., CartesianPosition[2],
		0., 0., 0., 1.;
	T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);

	double *T = new double[16];
    double *q_all = new double[48];
    Array vec;
    double cost = 0, Max_cost = 0;
    int num = 0;

	tm_jacobian::Matrix2DoubleArray(T_, T);
	num = tm_kinematics::inverse(T, q_all);
    if(num == 0){
        std::cout << "Solution number is 0" << num <<std::endl;
        std::cout << "FAIL" << num <<std::endl;
        return false;
    }
    int doNum = 0;
    do{
        cost = 0;
        for(int j = 0; j < 6; j++){
            cost += std::pow((q_all[doNum+j] - startJ[j]), 2);
            vec[j] = q_all[doNum+j];
        }
        if(cost < Max_cost || doNum == 0){
            Max_cost = cost;
            for(int j = 0; j < 6; j++)
                q_inv[j] = vec[j];
        }      
        doNum += 6;
    }
    while(doNum < num*6);

	delete[] T;
    delete[] q_all;
	return CheckJointLimit(q_inv);
}

bool GetQdfromLinearJacobian(Array curQ, Array EFF_Velocity, Array& qd)
{
    Eigen::Matrix<float, 6, 1> home, q;
    Eigen::Matrix<double, 6,1> JointSpeed;
    Eigen::Matrix<double, 6, 1> EFFSpeed;

    home     << 0, -PI*0.5, 0, PI*0.5, 0, 0;
    q        << curQ[0], curQ[1], curQ[2], curQ[3], curQ[4], curQ[5];
    EFFSpeed << EFF_Velocity[0], EFF_Velocity[1], EFF_Velocity[2], EFF_Velocity[3], EFF_Velocity[4], EFF_Velocity[5];
    q += home;

    Eigen::Matrix<double, 6, 6> Geometry_Jacobian = tm_jacobian::Forward_Jacobian_d(q);
    // Eigen::Matrix<double, 3, 6> Jacobian_123456   = Geometry_Jacobian.block<3,6>(0,0);

    Eigen::MatrixXd Geometry_Jacobian_inv;
    pinv_SVD(Geometry_Jacobian,Geometry_Jacobian_inv);
    JointSpeed = Geometry_Jacobian_inv * EFFSpeed;
    for (int i = 0; i < NUM_DOF; ++i)
        qd[i] = JointSpeed(i);

    return true;
}