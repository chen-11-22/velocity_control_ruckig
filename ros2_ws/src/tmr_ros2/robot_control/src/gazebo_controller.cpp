#include "gazebo_controller.h"

using namespace control_interface;

void gazebo_control::ruckig_state_manage(){
    if (ruckig_activate) {
        ruckig::Result ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
        if (ruckig_result != ruckig::Result::Finished) {
            auto& p = ruckig_output.new_velocity;
            std::vector<double> cmd_vel(NUM_DOF);
            for (size_t i=0; i < NUM_DOF; ++i)
            {
                cmd_vel.at(i) = p.at(i);
                current_joint_acl.at(i) = std::clamp(
                    ruckig_output.new_acceleration.at(i), -ruckig_input.max_acceleration.at(i), ruckig_input.max_acceleration.at(i));
            }
            // std::cout << ruckig_output.time << " " << p[0] << " " << p[1] << " " << p[2] 
            // << " " << p[3]  << " " << p[4] << " " << p[5] << std::endl;
            this->publish_vel(cmd_vel);
            ruckig_output.pass_to_input(ruckig_input);
            // std::copy_n(current_joint.begin(), NUM_DOF, ruckig_input.current_position.begin());
        } 
        else {
            printf("Target Reached!!\n");
            for (size_t i=0; i < NUM_DOF; ++i)
                printf("%f ",current_joint.at(i));
            printf("\n");
            // printf("real position\n");
            // for (size_t i=0; i < NUM_DOF; ++i)
            //     printf("%f ",current_joint.at(i));
            // printf("\n");
            // RCLCPP_INFO("Target Reached!!");
            // auto& p = ruckig_output.new_position;
            ruckig_activate = false;
            std::vector<double> cmd_vel(NUM_DOF, 0.0);
            std::fill(&EndPointJ[0], &EndPointJ[0] + 6, 0.0);
            std::fill(&EndPointC[0], &EndPointC[0] + 6, 0.0);
            // std::copy_n(p.begin(), NUM_DOF, current_joint.begin());
        }
    }
}

void gazebo_control::publish_vel(std::vector<double> vel_cmd){
    // printf("publish_vel\n");
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = vel_cmd;
    this->vel_pub->publish(message);
}

void gazebo_control::joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_msg) {
    // printf("joint_callback\n");
    // for (size_t i=0; i < NUM_DOF; ++i)
    //     current_joint.at(i) = joint_msg->position.at(i);
    //Because of gazebo
    current_joint[0] = joint_msg->position[0];
    current_joint[1] = joint_msg->position[1];
    current_joint[2] = joint_msg->position[4];
    current_joint[3] = joint_msg->position[2];
    current_joint[4] = joint_msg->position[3];
    current_joint[5] = joint_msg->position[5];
}

void gazebo_control::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    ruckig_ptr = std::make_unique<ruckig::Ruckig<NUM_DOF>>(0.04);
    printf("cmd_callback\n");
    EndPointC[0] = msg->linear.x;
    EndPointC[1] = msg->linear.y;
    EndPointC[2] = msg->linear.z;
    EndPointC[3] = msg->angular.x;
    EndPointC[4] = msg->angular.y;
    EndPointC[5] = msg->angular.z;

    getNextRuckigInput();
    ruckig_activate = true;
}

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

// double *T = new double[16];   
// double *q_q  = new double[6];
bool gazebo_control::GetQfromInverseKinematics(double* CartesianPosition, double *q_inv)
{
	Eigen::Matrix<float, 4, 4> T_;
	Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
	Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
	double *T = new double[16];
    double *q_all = new double[48];
    std::array<double, 6> vec;
    double cost = 0, Max_cost = 0;
    int num = 0;

	T_ << 0., 0., 0., CartesianPosition[0],
		0., 0., 0., CartesianPosition[1],
		0., 0., 0., CartesianPosition[2],
		0., 0., 0., 1.;

	T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);
	tm_jacobian::Matrix2DoubleArray(T_, T);
	num = tm_kinematics::inverse(T, q_all);
    if(num == 0){
        std::cout << "Solution number is 0" << num <<std::endl;
        std::cout << "FAIL" << num <<std::endl;
        return false;
    }
    int doNum = 0;
    do{
        std::cout << "solution number is  " << num <<std::endl;
        cost = 0;
        for(int j = 0; j < 6; j++){
            cost += std::pow((q_all[doNum+j] - current_joint[j]), 2);
            std::cout << q_all[doNum+j] << " ";
            vec[j] = q_all[doNum+j];
        }std::cout << std::endl;
        std::cout << "cost is " << cost <<std::endl;
        if(cost < Max_cost || doNum == 0){
            Max_cost = cost;
            for(int j = 0; j < 6; j++)
                q_inv[j] = vec[j];
        }      
        doNum += 6;
    }
    while(doNum < num*6);

    std::cout << "final cost: " << Max_cost << std::endl;
    for(int j = 0; j < 6; j++){
        std::cout << q_inv[j] << " ";
    }std::cout << std::endl;

	delete[] T;
    delete[] q_all;
	return CheckJointLimit(q_inv);
}


void gazebo_control::getNextRuckigInput()
{
    bool within_limit = true;
    within_limit = GetQfromInverseKinematics(EndPointC, EndPointJ);
    if (within_limit) {
        for (size_t joint = 0; joint < NUM_DOF; ++joint)
        {
            // Target state is the next waypoint
            ruckig_input.target_position.at(joint) = EndPointJ[joint];
            ruckig_input.target_velocity.at(joint) = 0.0;
            ruckig_input.target_acceleration.at(joint) = 0.0;
            printf("%f ",EndPointJ[joint]);
        }
        printf("\n");
        
        initializeRuckigState();
    } else {
        RCLCPP_WARN(get_logger(),"Joint position over limit, skip target");
    }
}

void gazebo_control::initializeRuckigState(){
    ruckig_input.control_interface = ruckig::ControlInterface::Position;
    std::copy_n(current_joint.begin(), NUM_DOF, ruckig_input.current_position.begin());
    std::copy_n(current_joint_vel.begin(), NUM_DOF, ruckig_input.current_velocity.begin());
    std::copy_n(current_joint_acl.begin(), NUM_DOF, ruckig_input.current_acceleration.begin());
    std::copy_n(max_velocities.begin(), NUM_DOF, ruckig_input.max_velocity.begin());
    std::copy_n(max_accelerations.begin(), NUM_DOF, ruckig_input.max_acceleration.begin());
    std::copy_n(max_jerks.begin(), NUM_DOF, ruckig_input.max_jerk.begin());
    // Initialize output data struct
    ruckig_output.new_position = ruckig_input.current_position;
    ruckig_output.new_velocity = ruckig_input.current_velocity;
    ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gazebo_control>(rclcpp::NodeOptions());
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    return EXIT_SUCCESS;
}
