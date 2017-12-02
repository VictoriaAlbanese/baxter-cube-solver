//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: iks_class.cpp
//
// Description: Implements a class which abstracts away 
// baxter's inverse kinematic solver
//
//////////////////////////////////////////////////////////////

#include "iks_class.hpp"

//////////////////////////////////////////////////////////////

// Public Member Functions

// DEFAULT CONSTRUCTOR
// Sets up a default endpoint, no ros setup
IKS::IKS() 
{
    this->point = Endpoint();
}

// CONSTRUCTOR
// Wait for an endpoint and does the ros setup, then get the iks
IKS::IKS(ros::NodeHandle handle) 
{
    this->point = Endpoint(handle);
	if (this->point.get_point().x < TABLE_CENTER) service_name = "ExternalTools/left/PositionKinematicsNode/IKService";
    else service_name = "ExternalTools/right/PositionKinematicsNode/IKService";

	this->client = handle.serviceClient<baxter_core_msgs::SolvePositionIK>(service_name);
	
	this->make_service_request();
	this->get_iks();
    this->iks_to_joint_command();
}

//////////////////////////////////////////////////////////////

// Private Helper Functions & Callbacks

// MAKE SERVICE REQUEST
// Makes the service request by setting up the endpoint
void IKS::make_service_request() 
{
    // Print the point
	ROS_INFO("(%f, %f, %f)", 
		this->point.pose.point.x, 
		this->point.pose.point.y, 
		this->point.pose.point.z);

    // Make the service request
    this->service.request.pose_stamp.resize(1);
    this->service.request.pose_stamp[0] = this->point;
    this->service.request.seed_mode = 2;
} 

// GET IKS
// Calls the service and gets the iks, or errors if there is none
void IKS::get_iks() 
{
    // Report if the call to the service fails and exit
    if (!this->client.call(this->service))
    {
        ROS_ERROR("Failure: unable to call service");
        exit(1);
    }

    // Report if there is no ik solution and exit
    if (!this->service.response.isValid[0]) 
    {
        ROS_ERROR("Failure: no valid ik joint solution");
        exit(1);
    }

    ROS_INFO("IKS FOUND...");
    this->solved_state = this->service.response.joints[0];
}

// IKS to JointCommand
// Converts the iks (of type sensor_msgs::JointState) to a joint command
void IKS::iks_to_joint_command() 
{
    baxter_core_msgs::JointCommand msg;
    msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

    if (this->arm_side == LEFT) 
    {
        msg.names.push_back("left_e0");
        msg.names.push_back("left_e1");
        msg.names.push_back("left_s0");
        msg.names.push_back("left_s1");
        msg.names.push_back("left_w0");
        msg.names.push_back("left_w1");
        msg.names.push_back("left_w2");

        msg.command.resize(msg.names.size());
        msg.command[0] = this->solved_state.position[2];
        msg.command[1] = this->solved_state.position[3];
        msg.command[2] = this->solved_state.position[0];
        msg.command[3] = this->solved_state.position[1];
        msg.command[4] = this->solved_state.position[4];
        msg.command[5] = this->solved_state.position[5];
        msg.command[6] = this->solved_state.position[6];
    }

    else
    {
        msg.names.push_back("right_e0");
        msg.names.push_back("right_e1");
        msg.names.push_back("right_s0");
        msg.names.push_back("right_s1");
        msg.names.push_back("right_w0");
        msg.names.push_back("right_w1");
        msg.names.push_back("right_w2");

        msg.command.resize(msg.names.size());
        msg.command[0] = this->solved_state.position[2];
        msg.command[1] = this->solved_state.position[3];
        msg.command[2] = this->solved_state.position[0];
        msg.command[3] = this->solved_state.position[1];
        msg.command[4] = this->solved_state.position[4];
        msg.command[5] = this->solved_state.position[5];
        msg.command[6] = this->solved_state.position[6];
    }

    this->orders = msg;
}

//////////////////////////////////////////////////////////////
