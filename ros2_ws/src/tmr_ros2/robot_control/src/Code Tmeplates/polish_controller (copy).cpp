/*
    1.Get current_joint, current_joint_velocity information from tm_driver by ros topic
    2.Refresh current_joint_acceleraton with ruckig computation.
    3.We don't need to set joint acceleraton command to tm_flow, so we won't get 
        current_joint_acceleraton from tm_driver.
    4.Every point to point use ruckig_move() to set velocity command to tm_flow by ruckig.
    5.NextWayPointRuckig() will decide the ruckig input data up to mode. 
*/

#include "polish_controller.h"
using namespace polish_control_interface;
using namespace kinematic_tool;

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

void robot_control::NextWayPointRuckig()
{
    // when use muti-point smooth control
    //>>>>>>>>>>>> refresh target data to ruckug input <<<<<<<<<<<<<<
    if(ControlMode == WPSTART){
        WP_data.J_ready = current_joint;
        ruckig_input.target_position = WP_data.J_first;
        std::fill(ruckig_input.target_velocity.begin(), ruckig_input.target_velocity.end(), 0);
        std::fill(ruckig_input.target_acceleration.begin(), ruckig_input.target_acceleration.end(), 0.0);
        ControlMode = WP2READY;
    }
    //the last point
    else if(WP_data.Index == WP_data.Len){
        ruckig_input.target_position = WP_data.J_ready;
        std::fill(ruckig_input.target_velocity.begin(), ruckig_input.target_velocity.end(), 0);
        std::fill(ruckig_input.target_acceleration.begin(), ruckig_input.target_acceleration.end(), 0.0);
        ruckig_input.minimum_duration = std::nullopt; //minimum_duration is std::optional<T>
        ControlMode = WPDONE;
    }
    else{
        auto index = WP_data.Index;
        ruckig_input.target_position = WP_data.J_ptr->at(index);
        ruckig_input.target_velocity = WP_data.Jv_ptr->at(index);
        ruckig_input.minimum_duration = WP_data.t[index];
        ++WP_data.Index;
        ControlMode = WPWORK;

        std::cout << "time is: " <<  WP_data.t[index] << std::endl;
        std::cout << "normmm is:" << normmm[index] << std::endl;
        std::cout << "target_cartition is: " << carti_p[index].transpose() << std::endl;
        std::cout << "target_velocity is: ";
        for(auto i: ruckig_input.target_velocity)
            std::cout << i << ", ";
        std::cout << std::endl;
        std::cout << "target_joint is: ";
        for(auto i: ruckig_input.target_position)
            std::cout << i << ", ";
        std::cout << std::endl;
        std::cout << "-------------------------------------------\n";

    }
    //>>>>>>>>>>>> refresh target data to ruckug input <<<<<<<<<<<<<<
    initializeRuckigState(); //refresh current data to ruckug
    try{
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
    if (ruckig_result != ruckig::Result::Finished) {
        auto& vel = ruckig_output.new_velocity;
        auto& acl = ruckig_output.new_acceleration;
        std::vector<double> cmd_vel(NUM_DOF);

        std::copy_n(vel.begin(), NUM_DOF, cmd_vel.begin());
        std::copy_n(acl.begin(), NUM_DOF, current_joint_acl.begin());
        //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
        auto Vrequest = std::make_shared<tm_msgs::srv::SetVelocity::Request>();
        Vrequest -> motion_type = 1;
        Vrequest -> velocity = cmd_vel;
        vel_client->async_send_request(Vrequest);
        //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
        ruckig_output.pass_to_input(ruckig_input);
    } 
    else {
        if(ControlMode == WP2READY || ControlMode == WPWORK){
            ControlMode = WPWAITNEXT;
        }
        else{
            std::vector<double> cmd_vel(NUM_DOF);
            std::fill(cmd_vel.begin(), cmd_vel.end(), 0.0);
            //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
            auto Vrequest = std::make_shared<tm_msgs::srv::SetVelocity::Request>();
            Vrequest -> motion_type = 1;
            Vrequest -> velocity = cmd_vel;
            vel_client->async_send_request(Vrequest);
            //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
            request_mode("spdmodeoff");
            ControlMode = FREE;
            PolishMode = false; 
        }   
    }
}

bool robot_control::request_mode(std::string RT_MODE)
{
    auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
    if(RT_MODE == "Vstart"){
        request->id = "Vstart";
        request->script = "ContinueVJog()";
    }
    else if(RT_MODE == "spdmodeoff"){
        request->id = "spdmodeoff";
        request->script = "StopContinueVmode()";
    }
    else{
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "FALSE requset type. Exiting.");
        return false;
    }
    //wait for server
    while (!client_sendscript->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto request_result = client_sendscript->async_send_request(request);

    return true;
}


void robot_control::joint_callback(const tm_msgs::msg::FeedbackState::SharedPtr joint_msg) 
{
    if(joint_msg->joint_pos.size() == 6){
        for (size_t i = 0; i < NUM_DOF; ++i) {
            current_joint_vel.at(i) =
                std::clamp(joint_msg->joint_vel.at(i), -ruckig_input.max_velocity.at(i), ruckig_input.max_velocity.at(i));
            current_joint.at(i) = joint_msg->joint_pos.at(i);
        }
    }

}

void robot_control::cartesian_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if(ControlMode != FREE) return;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "SEND cartesian DATA, USING VELOCITY CONTROL");
    ruckig_ptr = std::make_unique<ruckig::Ruckig<NUM_DOF>>(0.04);
    EndPointC[0] = msg->linear.x;
    EndPointC[1] = msg->linear.y;
    EndPointC[2] = msg->linear.z;
    EndPointC[3] = msg->angular.x;
    EndPointC[4] = msg->angular.y;
    EndPointC[5] = msg->angular.z;
    NextPointRuckig();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cartesian_cmd_callback");
    // printf("cartesian_cmd_callback\n");
    request_mode("Vstart");
    
    ControlMode = ONE;
}

void robot_control::joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
{
    if(ControlMode != FREE) return;

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "SEND JOINT DATA, USING POSITION CONTROL");
    std::vector<double> targetJ(6);
    // targetJ[0] = msg->position[0];
    // targetJ[1] = msg->position[1];
    // targetJ[2] = msg->position[4];
    // targetJ[3] = msg->position[2];
    // targetJ[4] = msg->position[3];
    // targetJ[5] = msg->position[5];
    targetJ[0] = -2.9863974736976435;
    targetJ[1] = -0.3730158911085439;
    targetJ[2] = 2.141343037143821;
    targetJ[3] = -0.5216982106863638;
    targetJ[4] = 1.6018664951264696;
    targetJ[5] = 0.1247393695345443;
    PositionRun(targetJ);
}

void robot_control::mode_callback(const std_msgs::msg::Bool::SharedPtr bool_msg)
{
    if(ControlMode != FREE) return;
    ruckig_ptr = std::make_unique<ruckig::Ruckig<NUM_DOF>>(0.04);
    if(!WayPointDataSet() && PolishMode) return;  
    initializeRuckigState();
    PolishMode = bool_msg->data;
    request_mode("Vstart");
    ControlMode = WPSTART;
}

void robot_control::NextPointRuckig()
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

// original
// bool robot_control::WayPointDataSet()
// {
//     //open the path.txt file
//     std::ifstream inputFile(TxtFileName);
//     if (!inputFile.is_open()) {
//         std::cout << "Can't open file:" << TxtFileName << std::endl;
//         return false;
//     }
//     WP_data.J_ptr = std::make_unique<PathList>();
//     WP_data.Jv_ptr = std::make_unique<PathList>();
//     std::vector<Eigen::VectorXd> CartesianPoint;
//     Array startJ = current_joint;

//     //open .txt file which save path
//     std::string line;
//     while (std::getline(inputFile, line)) 
//     {
//         Eigen::VectorXd PointC_Eigen(6);
//         std::istringstream iss(line);
//         Array PointJ;
//         double value, PointC[NUM_DOF];
//         bool within_limit = true;

//         //read Cartesian data(x,y,z,rx,ry,rz) from .txt
//         for(size_t index = 0; index < NUM_DOF; ++index){
//             iss >> value;
//             PointC[index] = value;
//             PointC_Eigen[index] = value;
//         } 
//         //convert Cartesian space data to joint space data
//         within_limit = GetQfromInverseKinematics(PointC, PointJ, startJ);
//         if (within_limit) { 
//             WP_data.J_ptr->push_back(PointJ);
//             CartesianPoint.push_back(PointC_Eigen);
//             startJ = PointJ;
//         }
//     } inputFile.close();
    
//     //compute time, velocity
//     for(size_t i = 1; i < CartesianPoint.size(); i++)
//     {
//         float Id_time = (CartesianPoint[i].head<3>() - CartesianPoint[i-1].head<3>()).norm()/Ideal_Velocity;
//         Eigen::VectorXd v(6);
//         // auto v = (CartesianPoint[i] - CartesianPoint[i-1]).array()/Id_time;
//         for(size_t D = 0; D < 6; D++){
//             if(D < 3)
//                 v[D] = (CartesianPoint[i][D] - CartesianPoint[i-1][D])/Id_time;
//             else{
//                 if(CartesianPoint[i][D]*CartesianPoint[i-1][D] >= 0){
//                     v[D] = (CartesianPoint[i][D] - CartesianPoint[i-1][D])/Id_time;
//                 }else{
//                     double no1, no2;
//                     if(CartesianPoint[i][D] < 0){
//                         no2 = CartesianPoint[i-1][D];
//                         no1 = 2*M_PI + CartesianPoint[i][D];
//                     }else{
//                         no2 = 2*M_PI + CartesianPoint[i-1][D];
//                         no1 = CartesianPoint[i][D];
//                     }
//                     v[D] = std::abs(CartesianPoint[i][D]-CartesianPoint[i-1][D]) < std::abs(no1-no2)
//                                 ?(CartesianPoint[i][D]-CartesianPoint[i-1][D]) :(no1-no2);
//                     v[D] /= Id_time;
//                 }    
//             }
//         }
//         Array Cartesian_Velocity = {v[0], v[1], v[2], v[3], v[4], v[5]}, qV;
//         WP_data.t.push_back(Id_time);
//         GetQdfromLinearJacobian(WP_data.J_ptr->at(i), Cartesian_Velocity, qV);
//         WP_data.Jv_ptr->push_back(qV);

//         normmm.push_back((CartesianPoint[i].head<3>() - CartesianPoint[i-1].head<3>()).norm());
//         carti_p.push_back(CartesianPoint[i]);
//     }
//     WP_data.J_first = WP_data.J_ptr->at(0);
//     WP_data.J_ptr->erase(WP_data.J_ptr->begin());
//     WP_data.Len = WP_data.J_ptr->size();
//     WP_data.Index = 0;

//     return true;
// }

//cartion veclocty (preV + befV) / 2
// bool robot_control::WayPointDataSet()
// {
//     //open the path.txt file
//     std::ifstream inputFile(TxtFileName);
//     if (!inputFile.is_open()) {
//         std::cout << "Can't open file:" << TxtFileName << std::endl;
//         return false;
//     }
//     WP_data.J_ptr = std::make_unique<PathList>();
//     WP_data.Jv_ptr = std::make_unique<PathList>();
//     std::vector<Eigen::VectorXd> CartesianPoint;
//     Array startJ = current_joint;

//     //open .txt file which save path
//     std::string line;
//     while (std::getline(inputFile, line)) 
//     {
//         Eigen::VectorXd PointC_Eigen(6);
//         std::istringstream iss(line);
//         Array PointJ;
//         double value, PointC[NUM_DOF];
//         bool within_limit = true;

//         //read Cartesian data(x,y,z,rx,ry,rz) from .txt
//         for(size_t index = 0; index < NUM_DOF; ++index){
//             iss >> value;
//             PointC[index] = value;
//             PointC_Eigen[index] = value;
//         } 
//         //convert Cartesian space data to joint space data
//         within_limit = GetQfromInverseKinematics(PointC, PointJ, startJ);
//         if (within_limit) { 
//             WP_data.J_ptr->push_back(PointJ);
//             CartesianPoint.push_back(PointC_Eigen);
//             startJ = PointJ;
//         }
//     } inputFile.close();
    
//     //compute time, velocity
//     std::vector<Eigen::VectorXd> CarVelSet;
//     for(size_t i = 1; i < CartesianPoint.size(); i++)
//     {
//         float Id_time = (CartesianPoint[i].head<3>() - CartesianPoint[i-1].head<3>()).norm()/Ideal_Velocity;
//         Eigen::VectorXd v(6);
//         for(size_t D = 0; D < 6; D++){
//             if(D < 3)
//                 v[D] = (CartesianPoint[i][D] - CartesianPoint[i-1][D])/Id_time;
//             else{
//                 if(CartesianPoint[i][D]*CartesianPoint[i-1][D] >= 0){
//                     v[D] = (CartesianPoint[i][D] - CartesianPoint[i-1][D])/Id_time;
//                 }else{
//                     double no1, no2;
//                     if(CartesianPoint[i][D] < 0){
//                         no2 = CartesianPoint[i-1][D];
//                         no1 = 2*M_PI + CartesianPoint[i][D];
//                     }else{
//                         no2 = 2*M_PI + CartesianPoint[i-1][D];
//                         no1 = CartesianPoint[i][D];
//                     }
//                     v[D] = std::abs(CartesianPoint[i][D]-CartesianPoint[i-1][D]) < std::abs(no1-no2)
//                                 ?(CartesianPoint[i][D]-CartesianPoint[i-1][D]) :(no1-no2);
//                     v[D] /= Id_time;
//                 }    
//             }
//         }
//         CarVelSet.push_back(v);

//         // Array Cartesian_Velocity = {v[0], v[1], v[2], v[3], v[4], v[5]}, qV;
//         WP_data.t.push_back(Id_time);
//         // GetQdfromLinearJacobian(WP_data.J_ptr->at(i), Cartesian_Velocity, qV);
//         // WP_data.Jv_ptr->push_back(qV);

//         normmm.push_back((CartesianPoint[i].head<3>() - CartesianPoint[i-1].head<3>()).norm());
//         carti_p.push_back(CartesianPoint[i]);
//     }

//     //V0_target = (V0_target + V1_target) / 2
//     for(size_t nw = 0; nw < CarVelSet.size(); nw++) {
//         Eigen::VectorXd v(6);
//         if(nw == CarVelSet.size()-1)
//             v = CarVelSet[nw].array()/2;
//         else
//             v = (CarVelSet[nw] + CarVelSet[nw+1]).array()/2;
//         Array Cartesian_Velocity = {v[0], v[1], v[2], v[3], v[4], v[5]}, qV;
//         GetQdfromLinearJacobian(WP_data.J_ptr->at(nw+1), Cartesian_Velocity, qV);
//         WP_data.Jv_ptr->push_back(qV);
//     }

//     WP_data.J_first = WP_data.J_ptr->at(0);
//     WP_data.J_ptr->erase(WP_data.J_ptr->begin());
//     WP_data.Len = WP_data.J_ptr->size();
//     WP_data.Index = 0;

//     return true;
// }

//compute in joint space
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
    std::vector<Eigen::VectorXd> CartesianPoint;
    Array startJ = current_joint;

    //open .txt file which save path
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
        within_limit = GetQfromInverseKinematics(PointC, PointJ, startJ);
        if (within_limit) { 
            WP_data.J_ptr->push_back(PointJ);
            CartesianPoint.push_back(PointC_Eigen);
            startJ = PointJ;
        }
    } inputFile.close();
    
    //compute time, velocity
    std::vector<Eigen::VectorXd> JointVelSet;
    for(size_t i = 1; i < CartesianPoint.size(); i++)
    {
        float Id_time = (CartesianPoint[i].head<3>() - CartesianPoint[i-1].head<3>()).norm()/Ideal_Velocity;
        Eigen::VectorXd v(6);
        for(size_t dof = 0; dof < 6; dof++){
            v[dof] = (WP_data.J_ptr->at(i).at(dof) - WP_data.J_ptr->at(i-1).at(dof))/Id_time;
        } 
        WP_data.t.push_back(Id_time);
        JointVelSet.push_back(v);
        // Array Joints_Velocity = {v[0], v[1], v[2], v[3], v[4], v[5]};
        // WP_data.Jv_ptr->push_back(Joints_Velocity);

        normmm.push_back((CartesianPoint[i].head<3>() - CartesianPoint[i-1].head<3>()).norm());
        carti_p.push_back(CartesianPoint[i]);
    }

    for(size_t nw = 0; nw < JointVelSet.size(); nw++) {
        Eigen::VectorXd v(6);
        if(nw == JointVelSet.size()-1)
            v = JointVelSet[nw].array()/2;
        else
            v = (JointVelSet[nw] + JointVelSet[nw+1]).array()/2;
        Array Joints_Velocity = {v[0], v[1], v[2], v[3], v[4], v[5]};
        WP_data.Jv_ptr->push_back(Joints_Velocity);
    }

    WP_data.J_first = WP_data.J_ptr->at(0);
    WP_data.J_ptr->erase(WP_data.J_ptr->begin());
    WP_data.Len = WP_data.J_ptr->size();
    WP_data.Index = 0;

    return true;
}



void robot_control::initializeRuckigState()
{
    ruckig_input.control_interface = ruckig::ControlInterface::Position;
    std::copy_n(current_joint.begin(), NUM_DOF, ruckig_input.current_position.begin());
    std::copy_n(current_joint_vel.begin(), NUM_DOF, ruckig_input.current_velocity.begin());
    std::copy_n(current_joint_acl.begin(), NUM_DOF, ruckig_input.current_acceleration.begin());
    // Initialize output data struct
    ruckig_output.new_position = ruckig_input.current_position;
    ruckig_output.new_velocity = ruckig_input.current_velocity;
    ruckig_output.new_acceleration = ruckig_input.current_acceleration;
}

void robot_control::PositionRun(std::vector<double> targetJ)
{
    auto p_request = std::make_shared<tm_msgs::srv::SetPositions::Request>();
    p_request->motion_type = tm_msgs::srv::SetPositions::Request::PTP_J;
    p_request->positions = targetJ;
    p_request->velocity = 1;
    p_request->acc_time = 0.4;
    p_request->blend_percentage = 10;
    p_request->fine_goal  = false;

    while (!p_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto p_result = p_client->async_send_request(p_request);
}


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
    for (int i = 0; i < m ; ++i){
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


int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    std::string fileName = "/home/isci/colcon_ws/WayPoints_HEE.txt";
    auto node = std::make_shared<robot_control>(rclcpp::NodeOptions(), fileName);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),8,true);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    
    return EXIT_SUCCESS;
}
