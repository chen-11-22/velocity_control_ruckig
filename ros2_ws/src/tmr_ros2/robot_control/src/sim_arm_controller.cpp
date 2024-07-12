#include "sim_arm_controller.h"

using namespace arm_control_interface;
using namespace kinematic_tool;

//------------------------------arm_control_interface------------------------------
void robot_control::ruckig_state_manage()
{
    switch (ControlMode) 
    {
        case WPSTART:
        case WPWAITNEXT:
            NextWayPointRuckig();
            break;

        case ONE:
        case WP2READY:
        case WPWORK:
        case WPDONE:
            ruckig_move();
            break;

        case FREE:
            break;
    }
}

void robot_control::initializeRuckigState()
{
    ruckig_input.control_interface = ruckig::ControlInterface::Position;
    //Copy current_joint to current_position
    std::copy_n(current_joint.begin(), NUM_DOF, ruckig_input.current_position.begin());
    std::copy_n(current_joint_vel.begin(), NUM_DOF, ruckig_input.current_velocity.begin());
    std::copy_n(current_joint_acl.begin(), NUM_DOF, ruckig_input.current_acceleration.begin());
    //Initialize output data struct 
    //Assign current status to output
    ruckig_output.new_position = ruckig_input.current_position;
    ruckig_output.new_velocity = ruckig_input.current_velocity;
    ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

void robot_control::NextWayPointRuckig() //velocity control
{
    if(ControlMode == WPSTART){
        //WP_data is WayPointData, struct => J_ptr,Jv_ptr,t,J_first,J_ready,Len,Index
        //Setting initial state
        WP_data.J_ready = current_joint;
        ruckig_input.target_position = WP_data.J_first;
        //target velocity, accerleration is set to 0
        std::fill(ruckig_input.target_velocity.begin(), ruckig_input.target_velocity.end(), 0);
        std::fill(ruckig_input.target_acceleration.begin(), ruckig_input.target_acceleration.end(), 0.0);
        ControlMode = WP2READY;
    }
    else if(WP_data.Index == WP_data.Len){
        //The arm is already moved to last point
        ruckig_input.target_position = WP_data.J_ready;
        //The end state velocity, acceleration is set to 0
        std::fill(ruckig_input.target_velocity.begin(), ruckig_input.target_velocity.end(), 0);
        std::fill(ruckig_input.target_acceleration.begin(), ruckig_input.target_acceleration.end(), 0.0);
        ruckig_input.minimum_duration = std::nullopt; //minimum_duration is std::optional<T>
        //nullopt is "no value" or "empty"
        ControlMode = WPDONE;
    }
    else{
        //the arm is stll got waypoint need to move to
        auto index = WP_data.Index;
        ruckig_input.target_position = WP_data.J_ptr->at(index);
        ruckig_input.target_velocity = WP_data.Jv_ptr->at(index);
        ruckig_input.minimum_duration = WP_data.t[index];
        ++WP_data.Index;
        ControlMode = WPWORK;
    }
    initializeRuckigState(); //initialize ruckig state
    try{
        //validate_input(InputParameter, false, true)
        ruckig_ptr->validate_input(ruckig_input, false, true);
    }
    catch(const std::exception& e){
        std::cout << e.what() << std::endl;
        ruckig_input.target_position = WP_data.J_ready;
        std::fill(ruckig_input.target_velocity.begin(), ruckig_input.target_velocity.end(), 0);
        std::fill(ruckig_input.target_acceleration.begin(), ruckig_input.target_acceleration.end(), 0.0);
        ruckig_input.minimum_duration = std::nullopt; //minimum_duration is std::optional<T>
        ControlMode = WPDONE;
    }
}

void robot_control::ruckig_move()
{
    ruckig::Result ruckig_result = ruckig_ptr->update(ruckig_input, ruckig_output);
    // if (ruckig_result != ruckig::Result::Finished && ruckig_result == ruckig::Result::Working) {
    if (ruckig_result != ruckig::Result::Finished) {
        auto& vel = ruckig_output.new_velocity;

        std::vector<double> cmd_vel(NUM_DOF);
        std::copy_n(vel.begin(), NUM_DOF, cmd_vel.begin());
        this->publish_vel(cmd_vel);
        ruckig_output.pass_to_input(ruckig_input);
    } 
    else {//ruckig_result == ruckig::Result::Finished
        if(ControlMode == WP2READY || ControlMode == WPWORK){
            
            auto& vel = ruckig_output.new_velocity;
            auto& acl = ruckig_output.new_acceleration;
            std::copy_n(vel.begin(), NUM_DOF, current_joint_vel.begin());
            std::copy_n(acl.begin(), NUM_DOF, current_joint_acl.begin());

            //// Exxxxxxxxxxxxxxxxx data ////
            if(ControlMode == WP2READY && Move){
                exdata.time.clear();
                exdata.j.clear();
                exdata.jv.clear();
                start_time = now();
            }
            end_time = now();
            rclcpp::Duration duration = end_time - start_time;
            double duration_ms = duration.nanoseconds() / 1e9;
            exdata.time.push_back(duration_ms);
            exdata.j.push_back(current_joint);
            exdata.jv.push_back(current_joint_vel);
            // RCLCPP_INFO(get_logger(), "current_joint_vel is %.2f %2f %2f %2f %2f %2f", 
            //             current_joint_vel.at(0), current_joint_vel.at(1),
            //             current_joint_vel.at(2), current_joint_vel.at(3),
            //             current_joint_vel.at(4), current_joint_vel.at(5));
            //// Exxxxxxxxxxxxxxxxx data ////
            ControlMode = WPWAITNEXT;

        }
        else{
            //// Exxxxxxxxxxxxxxxxx data ////
            if(Move){
                RCLCPP_INFO(get_logger(), "Moving is done.");
                OutPutEx2F();
            }
            //// Exxxxxxxxxxxxxxxxx data ////

            std::vector<double> cmd_vel(NUM_DOF);
            std::fill(cmd_vel.begin(), cmd_vel.end(), 0.0);
            this->publish_vel(cmd_vel);
            ControlMode = FREE;
            Move = false;
            std::cout << "Reach target joint:";
            std::cout << current_joint[0] << " " << current_joint[1] << " " << current_joint[2] << " ";
            std::cout << current_joint[3] << " " << current_joint[4] << " " << current_joint[5] << " ";
            std::cout << "\n------------------------------------------" << std::endl;
        }   
    }
}

void robot_control::publish_vel(std::vector<double> vel_cmd){
    auto message = std_msgs::msg::Float64MultiArray();
    message.data = vel_cmd;
    this->vel_pub->publish(message);
}

void robot_control::joint_callback(const sensor_msgs::msg::JointState::SharedPtr joint_msg) {

    //Because of gazebo
    if(joint_msg->position[0] == 0 && joint_msg->position[1] ==0 &&
        joint_msg->position[2] == 0 && joint_msg->position[3] ==0 &&
        joint_msg->position[4] == 0 && joint_msg->position[5] ==0 ) 
    return;
    current_joint[0] = joint_msg->position[0];
    current_joint[1] = joint_msg->position[1];
    current_joint[2] = joint_msg->position[4];
    current_joint[3] = joint_msg->position[2];
    current_joint[4] = joint_msg->position[3];
    current_joint[5] = joint_msg->position[5];
}

void robot_control::cart_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(Move) return;

    ruckig_ptr = std::make_unique<ruckig::Ruckig<NUM_DOF>>(0.04);

    //cart
    // Arm will reach new accepting topic's point
    printf("cart_cmd_callback\n");
    EndPointC[0] = msg->linear.x;
    EndPointC[1] = msg->linear.y;
    EndPointC[2] = msg->linear.z;
    EndPointC[3] = msg->angular.x;
    EndPointC[4] = msg->angular.y;
    EndPointC[5] = msg->angular.z;

    // Arm will reach the current topic point, then continue to move to next topic point
    // if(ControlMode == TaskMode::FREE)
    // {
    //     printf("cart_cmd_callback\n");
    //     EndPointC[0] = msg->linear.x;
    //     EndPointC[1] = msg->linear.y;
    //     EndPointC[2] = msg->linear.z;
    //     EndPointC[3] = msg->angular.x;
    //     EndPointC[4] = msg->angular.y;
    //     EndPointC[5] = msg->angular.z;
    // }
    
    NextPointRuckig();
    ControlMode = ONE;
}

void robot_control::joint_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(Move) return;

    ruckig_ptr = std::make_unique<ruckig::Ruckig<NUM_DOF>>(0.04);
    
    //joint
    // Arm will reach new accepting topic's point
    printf("joint_cmd_callback\n");
    EndPointJ[0] = msg->linear.x;
    EndPointJ[1] = msg->linear.y;
    EndPointJ[2] = msg->linear.z;
    EndPointJ[3] = msg->angular.x;
    EndPointJ[4] = msg->angular.y;
    EndPointJ[5] = msg->angular.z;

    // Arm will reach the current topic point, then continue to move to next topic point
    // if(ControlMode == TaskMode::FREE) 
    // {
    //     printf("joint_cmd_callback\n");
    //     EndPointJ[0] = msg->linear.x;
    //     EndPointJ[1] = msg->linear.y;
    //     EndPointJ[2] = msg->linear.z;
    //     EndPointJ[3] = msg->angular.x;
    //     EndPointJ[4] = msg->angular.y;
    //     EndPointJ[5] = msg->angular.z;
    // }

    //NextPointRuckig
    ruckig_input.target_position = EndPointJ;
    std::fill(ruckig_input.target_velocity.begin(), ruckig_input.target_velocity.end(), 0.0);
    std::fill(ruckig_input.target_acceleration.begin(), ruckig_input.target_acceleration.end(), 0.0);
    initializeRuckigState();

    ControlMode = ONE;
}

// compute origin
bool robot_control::WayPointDataSet()
{
    //open the path.txt file
    std::ifstream inputFile(TxtFileName);
    if (!inputFile.is_open()) {
        std::cout << "Can't open file:" << TxtFileName << std::endl;
        return false;
    }
    WP_data.J_ptr = std::make_unique<PathList>();
    WP_data.Jv_ptr = std::make_unique<PathList>();
    std::vector<Eigen::VectorXd> CartesianPoint; //VectorXd row coloum
    Array startJ = current_joint;

    //open .txt file which save path
    //read cartesian path from txt and covert into joint path
    std::string line;
    while (std::getline(inputFile, line)) 
    {
        Eigen::VectorXd PointC_Eigen(6);
        std::istringstream iss(line);
        Array PointJ;
        double value, PointC[NUM_DOF];
        bool within_limit = true;

        //read Cartesian data(x,y,z,rx,ry,rz) from .txt
        for(size_t index = 0; index < NUM_DOF; ++index){
            iss >> value;
            PointC[index] = value;
            PointC_Eigen[index] = value;
        } 
        //convert Cartesian space data to joint space data
        //If IK has solution
        within_limit = GetQfromInverseKinematics(PointC, PointJ, startJ);
        if (within_limit) { 
            WP_data.J_ptr->push_back(PointJ);
            CartesianPoint.push_back(PointC_Eigen);
            startJ = PointJ;
        }
    } inputFile.close();
    
    //compute time, velocity
    for(size_t i = 1; i < CartesianPoint.size(); i++)
    {
        float Id_time = (CartesianPoint[i].head<3>() - CartesianPoint[i-1].head<3>()).norm()/Ideal_Velocity;
        Eigen::VectorXd v(6);
        // auto v = (CartesianPoint[i] - CartesianPoint[i-1]).array()/Id_time;
        for(size_t D = 0; D < 6; D++){
            if(D < 3)
                v[D] = (CartesianPoint[i][D] - CartesianPoint[i-1][D])/Id_time;
            else{
                if(CartesianPoint[i][D]*CartesianPoint[i-1][D] >= 0){
                    v[D] = (CartesianPoint[i][D] - CartesianPoint[i-1][D])/Id_time;
                }else{
                    double no1, no2;
                    if(CartesianPoint[i][D] < 0){
                        no2 = CartesianPoint[i-1][D];
                        no1 = 2*M_PI + CartesianPoint[i][D];
                    }else{
                        no2 = 2*M_PI + CartesianPoint[i-1][D];
                        no1 = CartesianPoint[i][D];
                    }
                    v[D] = std::abs(CartesianPoint[i][D]-CartesianPoint[i-1][D]) < std::abs(no1-no2)
                                ?(CartesianPoint[i][D]-CartesianPoint[i-1][D]) :(no1-no2);
                    v[D] /= Id_time;
                }    
            }
        }
        Array Cartesian_Velocity = {v[0], v[1], v[2], v[3], v[4], v[5]}, qV;
        WP_data.t.push_back(Id_time);
        GetQdfromLinearJacobian(WP_data.J_ptr->at(i), Cartesian_Velocity, qV);

        std::cout << CartesianPoint[i-1].transpose();
        std::cout << std::endl;
        std::cout << CartesianPoint[i].transpose();
        std::cout << std::endl;
        for(auto i :Cartesian_Velocity)
            std::cout << i << " ";
        std::cout << std::endl;
        for(auto i :qV)
            std::cout << i << " ";
        std::cout << std::endl;
        std::cout<< "---------------------------------------" << std::endl;

        WP_data.Jv_ptr->push_back(qV);
        Car_path.push_back(CartesianPoint[i]);
    }
    WP_data.J_first = WP_data.J_ptr->at(0);
    WP_data.J_ptr->erase(WP_data.J_ptr->begin());
    WP_data.Len = WP_data.J_ptr->size();
    WP_data.Index = 0;

    return true;
}

void robot_control::mode_callback(const std_msgs::msg::Bool::SharedPtr bool_msg)
{
    ruckig_ptr = std::make_unique<ruckig::Ruckig<NUM_DOF>>(0.04);
    if(!WayPointDataSet() && Move) return;  
    initializeRuckigState();
    Move = bool_msg->data;
    ControlMode = WPSTART;
}

void robot_control::NextPointRuckig() //point to point
{ 
    bool within_limit = true;
    within_limit = GetQfromInverseKinematics(EndPointC, EndPointJ, current_joint);
    if (within_limit) {
        ruckig_input.target_position = EndPointJ;
        std::fill(ruckig_input.target_velocity.begin(), ruckig_input.target_velocity.end(), 0.0);
        std::fill(ruckig_input.target_acceleration.begin(), ruckig_input.target_acceleration.end(), 0.0);
        initializeRuckigState();
    } else {
        RCLCPP_WARN(get_logger(),"Joint position over limit, skip target");
    }
}

void robot_control::OutPutEx2F()
{
    size_t datalen = exdata.time.size();
    std::ofstream outputFile;

    //write joint data 
    outputFile.open(TxtOutPutFileName + "exp_joint.txt");
    if (outputFile.is_open()){
        for (size_t index = 0; index < datalen; index++){
            outputFile << exdata.time[index] << " ";
            for (auto dof_j: exdata.j[index])
                outputFile << dof_j << " ";
            outputFile << std::endl;
        }
        outputFile.close();
        std::cout << "Output File saved: " << TxtOutPutFileName + "exp_joint.txt" << std::endl;
    }
    else
        std::cerr << "Unable to open file: " << TxtOutPutFileName + "exp_joint.txt" << std::endl;

    //write joint velocity data 
    outputFile.open(TxtOutPutFileName + "exp_joint_velocity.txt");
    if (outputFile.is_open()){
        for (size_t index = 0; index < datalen; index++){
            outputFile << exdata.time[index] << " ";
            for (auto dof_jv: exdata.jv[index])
                outputFile << dof_jv << " ";
            outputFile << std::endl;
        }
        outputFile.close();
        std::cout << "Output File saved: " << TxtOutPutFileName + "exp_joint_velocity.txt" << std::endl;
    }
    else
        std::cerr << "Unable to open file: " << TxtOutPutFileName + "exp_joint_velocity.txt" << std::endl;
}

//------------------------------kinematic_tool------------------------------
bool kinematic_tool::GetQfromInverseKinematics(double *CartesianPosition, Array &q_inv, Array startJ)
{
	Eigen::Matrix<float, 4, 4> T_;
	Eigen::AngleAxisf yawAngle(CartesianPosition[5], Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf pitchAngle(CartesianPosition[4], Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
	Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
	Eigen::Matrix<float, 3, 3> RotationMatrix = q.matrix();
    // std::cout << RotationMatrix << std::endl;
    // std::cout << "--------------------------\n";
    T_ << 0., 0., 0., CartesianPosition[0],
		0., 0., 0., CartesianPosition[1],
		0., 0., 0., CartesianPosition[2],
		0., 0., 0., 1.;
	T_.block<3, 3>(0, 0) = RotationMatrix.block<3, 3>(0, 0);

	double *T = new double[16];
    double *q_all = new double[48];
    Array vec;
    double cost = 0, Max_cost = 0;
    int SolNum = 0, doNum = 0;

	tm_jacobian::Matrix2DoubleArray(T_, T);
	SolNum = tm_kinematics::inverse(T, q_all);
    if(SolNum == 0){
        std::cout << "Solution number is 0" << SolNum <<std::endl;
        std::cout << "FAIL" << SolNum <<std::endl;
        return false;
    }
    while(doNum < SolNum*6){
        cost = 0;
        for(int j = 0; j < 6; j++){
            cost += std::pow((q_all[doNum+j] - startJ[j]), 2);
            vec[j] = q_all[doNum+j];
        }
        if(cost <= Max_cost || doNum == 0){
            Max_cost = cost;
            for(int j = 0; j < 6; j++)
                q_inv[j] = vec[j];
        }
        doNum += 6;
    }

	delete[] T;
    delete[] q_all;
	return CheckJointLimit(q_inv);
}

bool kinematic_tool::GetQdfromLinearJacobian(Array curQ, Array EFF_Velocity, Array& qd)
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

bool kinematic_tool::pinv_SVD(const Eigen::MatrixXd &J, Eigen::MatrixXd &invJ)
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

bool kinematic_tool::CheckJointLimit(Array q)
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


void read_config(std::string &pathName, std::string &expfile)
{
    std::ifstream cFile("/home/isci/ros2_ws/exp_config.txt");
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
        if (name == "pathFile") pathName = value;
        else if (name == "exp_output") expfile = value;
    }   
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    std::string fileName, expfile_name;
    read_config(fileName, expfile_name);
    auto node = std::make_shared<robot_control>(rclcpp::NodeOptions(), fileName, expfile_name);

    //executors
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
