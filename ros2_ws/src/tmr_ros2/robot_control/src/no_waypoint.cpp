#include <iostream>
#include <ruckig/ruckig.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/set_velocity.hpp"
#include "tm_msgs/srv/set_positions.hpp"
#include "tm_msgs/srv/send_script.hpp"

#define NUMBER_OF_DOFS 6

using namespace ruckig;
using namespace std::chrono_literals;
using Vector = std::array<double, 6>;

std::shared_ptr<rclcpp::Node> node_sendscript;
std::shared_ptr<rclcpp::Node> node_setvel;
std::shared_ptr<rclcpp::Node> node_setp;

rclcpp::Client<tm_msgs::srv::SetVelocity>::SharedPtr vel_client;
rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr p_client;
rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client_sendscript;

typedef struct{
    std::vector<double> StartPoint;
    std::vector<double> EndPoint;
    bool WayPointControl;
    std::vector<Vector> WayPoint;
}RuckigInput;

bool RukigPositionRun(RuckigInput IP_state, double SynTime);
bool RukigPositionRun(RuckigInput IP_state, double MVel);
bool TOpRun(std::vector<double> target);
bool request_mode(std::string RT_MODE);


bool request_mode(std::string RT_MODE)
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
    if (rclcpp::spin_until_future_complete(node_sendscript, request_result) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
        if(request_result.get()->ok){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"SUCESS TO SEND SCRIPT");
        } 
        else {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"FAILED TO SEND SCRIPT");
        }
    } 
    else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }

    return true;
}

bool RukigPositionRun(RuckigInput IP_state, double SynTime)
{
    // Create instances: the Ruckig OTG as well as input and output parameters
    Ruckig<6> otg {0.040};  // control cycle
    InputParameter<6> IP;
    OutputParameter<6> OP;

    //send tm5 request
    request_mode("Vstart");
    auto Vrequest = std::make_shared<tm_msgs::srv::SetVelocity::Request>();
    Vrequest -> motion_type = 1;
    std::vector<double> VelocityCommand(6);

    // Set input parameters
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
	{
        IP.current_position[i] = IP_state.StartPoint[i];
        IP.current_velocity[i] = 0;
        IP.current_acceleration[i] = 0;
        IP.target_position[i] = IP_state.EndPoint[i];
        IP.target_velocity[i] = 0;
        IP.target_acceleration[i] = 0;
        IP.max_acceleration[i] = 0.5*40;
        IP.max_jerk[i] = 100;
        IP.max_velocity[i] = 3;
    }
    IP.minimum_duration = SynTime;


    // Generate the trajectory within the control loop
    std::cout << "t | p1 | p2 | p3" << std::endl;
    while (otg.update(IP, OP) == Result::Working) 
    {
        while (!vel_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), 
                "service not available, waiting again...");
        }

        // The area execution in 25ms real time sharp
        auto& p = OP.new_velocity;
        std::cout << OP.time << " " << p[0] << " " << p[1] << " " << p[2] 
            << " " << p[3]  << " " << p[4] << " " << p[5] << std::endl;

        VelocityCommand = { p[0], p[1], p[2], p[3], p[4], p[5] };

        //>>>>>>>>>>>> Send Velocity to TM-Driver <<<<<<<<<<<<<<
        Vrequest -> velocity = VelocityCommand;
        auto Vresult = vel_client->async_send_request(Vrequest);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_setvel, Vresult) == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
            if( Vresult.get()-> ok )
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"SEND VELOCITY SUCESS");
            else
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"SEND VELOCITY FAIL");
        }
        else 
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");

        OP.pass_to_input(IP);
        usleep(40*1000);
    }

    std::cout << "Trajectory duration: " << OP.trajectory.get_duration() << " [s]." << std::endl;
    request_mode("spdmodeoff");

    return true;
}

bool RukigWayPointRun(RuckigInput IP_state, double MVel)
{
    // Create instances: the Ruckig OTG as well as input and output parameters
    const double control_cycle {0.040};
    const size_t max_number_of_waypoints {10};  // for memory allocation
    Ruckig<NUMBER_OF_DOFS> otg {control_cycle, max_number_of_waypoints};  // control cycle
    InputParameter<NUMBER_OF_DOFS> IP;
    OutputParameter<NUMBER_OF_DOFS> OP {max_number_of_waypoints};
    
    //send tm5 request
    request_mode("Vstart");
    auto Vrequest = std::make_shared<tm_msgs::srv::SetVelocity::Request>();
    Vrequest-> motion_type = 1;
    std::vector<double> VelocityCommand(6);

    // Set input parameters
    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
	{
        IP.current_position[i] = IP_state.StartPoint[i];
        IP.current_velocity[i] = 0;
        IP.current_acceleration[i] = 0;

        IP.target_position[i] = IP_state.EndPoint[i];
        IP.target_velocity[i] = 0;
        IP.target_acceleration[i] = 0;
        
        IP.max_acceleration[i] = 1;
        IP.max_jerk[i] = 3;
        IP.max_velocity[i] = MVel;
    }

    for(auto wp: IP_state.WayPoint)
        IP.intermediate_positions.push_back(wp);
    IP.interrupt_calculation_duration = 500; // [µs]

    // Generate the trajectory within the control loop
    std::cout << "t | p1 | p2 | p3" << std::endl;
    while (otg.update(IP, OP) == Result::Working) 
    {
        while (!vel_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), 
                "service not available, waiting again...");
        }

        if (OP.new_calculation) {
            std::cout << "Updated the trajectory:" << std::endl;
            std::cout << "  Reached target position in " << OP.trajectory.get_duration() << " [s]." << std::endl;
            std::cout << "  Calculation in " << OP.calculation_duration << " [µs]." << std::endl;
        }

        // The area execution in 40ms real time sharp
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
        Vrequest -> velocity = VelocityCommand;
        auto Vresult = vel_client->async_send_request(Vrequest);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_setvel, Vresult) == rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->ok);
            if( Vresult.get()-> ok )
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"SEND VELOCITY SUCESS");
            else
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"SEND VELOCITY FAIL");
        }
        else 
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");

        OP.pass_to_input(IP);
        usleep(40*1000);
    }

    std::cout << "Trajectory duration: " << OP.trajectory.get_duration() << " [s]." << std::endl;
    request_mode("spdmodeoff");

    return true;
}

bool TOpRun(std::vector<double> target)
{
    auto p_request = std::make_shared<tm_msgs::srv::SetPositions::Request>();
    p_request->motion_type = tm_msgs::srv::SetPositions::Request::PTP_J;

    for(auto i: target)
        p_request->positions.push_back(i);

    p_request->velocity = 1;//rad/s
    p_request->acc_time = 0.4;
    p_request->blend_percentage = 10;
    p_request->fine_goal  = false;

    while (!p_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto p_result = p_client->async_send_request(p_request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_setp, p_result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        if(p_result.get()->ok){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"SEND POSITION SUCESS");
        } else{
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"SEND POSITION FAIL");
        }

    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }

    return true;
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    node_sendscript = rclcpp::Node::make_shared("demo_send_script");
    node_setvel = rclcpp::Node::make_shared("demo_set_velocity");
    node_setp = rclcpp::Node::make_shared("demo_set_positions");

    //request for use spdmode
    client_sendscript = node_sendscript->create_client<tm_msgs::srv::SendScript>("send_script");
    //set robot velocity
    vel_client = node_setvel->create_client<tm_msgs::srv::SetVelocity>("set_velocity");
    p_client = node_setp->create_client<tm_msgs::srv::SetPositions>("set_positions");
    
    RuckigInput A;
    A.StartPoint = {-1.4914166826053952, 0.01331910839432627, 1.8589293177688586, -0.3845250413649439, 1.4930353519159454, 0.11209542137052625};
    A.EndPoint = {-0.6700550586894625, -0.2906813032420315, 1.9125914811017373, -0.5967474217853418, 2.0402833974683117, 0.11214390754712251};
    A.WayPointControl = true;

    std::vector<double> target1(6);
    std::vector<double> target2(6);
    std::vector<double> target3(6);
    target1 = {-1.4914166826053952, 0.01331910839432627, 1.8589293177688586, -0.3845250413649439, 1.4930353519159454, 0.11209542137052625};
    target2 = {-1.1216667258101565, -0.2321916943801935, 1.9128528703627286, -0.7338430265400131, 1.4930549261499575, 0.11217300258203163};
    target3 = {-0.6700550586894625, -0.2906813032420315, 1.9125914811017373, -0.5967474217853418, 2.0402833974683117, 0.11214390754712251};

    // std::vector<double> w_point(6);
    Vector w_point = {-1.1216667258101565, -0.2321916943801935, 1.9128528703627286, -0.7338430265400131, 1.4930549261499575, 0.11217300258203163};
 
    A.WayPoint.push_back(w_point);
    // RukigWayPointRun(A, 0.2);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"ROBOT CONTROL IS READY");
    char cstr[512];
    while(rclcpp::ok()){
        memset(cstr, 0, 512);
        fgets(cstr, 512, stdin);
        int n = (int)strlen(cstr);
        if(n > 0){
            if(cstr[n - 1] == '\n')
                cstr[n - 1] = '\0';
        }

        if(strncmp(cstr, "quit", 4) == 0){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"bye bye");
            // printf("bye bye\n");
            break;
        }
        else if(strncmp(cstr, "point1", 6) == 0){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Go to start point");
            // printf("Go to start point\n");
            TOpRun(target1);
        }
        else if(strncmp(cstr, "point2", 6) == 0){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Go to start point");
            // printf("Go to start point\n");
            TOpRun(target2);
        }
        else if(strncmp(cstr, "point3", 6) == 0){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Go to start point");
            // printf("Go to start point\n");
            TOpRun(target3);
        }
        else if(strncmp(cstr, "vtask", 5) == 0){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Do task");
            // printf("Go to start point\n");
            RukigWayPointRun(A, 0.08);
        }
        else if(strncmp(cstr, "vvtask", 6) == 0){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Do task");
            // printf("Go to start point\n");
            RukigPositionRun(A, 5);
        }
        else if(strncmp(cstr, "ptask", 4) == 0){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Do task");
            // printf("Go to start point\n");
            TOpRun(target1);
            TOpRun(target2);
            TOpRun(target3);
        }
    }
    
    rclcpp::shutdown();
    return 0;
}
