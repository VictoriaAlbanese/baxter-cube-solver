//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: iks_class.cpp
//
// Description: Declares a class which abstracts away 
// baxter's inverse kinematic solver
//
//////////////////////////////////////////////////////////////

#include "iks_class.hpp"

//////////////////////////////////////////////////////////////

// Public Methods

// DEFAULT CONSTRUCTOR
// does not initialize ros or the endpoint
IKS::IKS() 
{
    this->initialized_ = false;
    
    this->arm_side = RIGHT;
    this->endpoint = Endpoint();
    this->baxter = FaceDisplay();
}

// CONSTRUCTOR
// does the ros initialization but 
// does not initialize the endpoint
IKS::IKS(ros::NodeHandle handle, bool arm_side) 
{
    this->initialized_ = false;
    
    this->handle = handle;
    this->baxter = FaceDisplay(handle);
    this->arm_side = arm_side;
    
    this->endpoint = Endpoint(this->handle);
    
    string service_name;
    if (this->arm_side == LEFT) service_name = "ExternalTools/left/PositionKinematicsNode/IKService";
    else service_name = "ExternalTools/right/PositionKinematicsNode/IKService";

	this->client = handle.serviceClient<baxter_core_msgs::SolvePositionIK>(service_name);
    this->kill_pub = handle.advertise<std_msgs::Bool>("kill_cloud", 10);
}

// FIND CUBE FUNCTION 
// initialize the endpoint, the location 
// at which the cube resides
void IKS::find_cube() 
{
    //this->endpoint = Endpoint(this->handle);
    this->initialized_ = this->endpoint.initialized();
}

// GET ORDERS FUNCTION
// waits for the endpoint to be initialized
// then gets iks and returns orders to the arms to get there
baxter_core_msgs::JointCommand IKS::get_orders() 
{
    this->make_service_request();
	this->get_iks();
    this->iks_to_joint_command();

    return this->orders;
}

// KILL CLOUD FUNCTION
// when this function is called, it signals that the point 
// cloud is no longer needed; a message is published to a special 
// topic that will be read by the cloud class, and will cause 
// the pcl_node to exit gracefully 
void IKS::kill_cloud() 
{
    std_msgs::Bool kill;
    kill.data = true;
    this->kill_pub.publish(kill);
}

//////////////////////////////////////////////////////////////

// Private Methods

// SERVICE REQUEST FUNCTION
// makes the service request by setting up the endpoint
void IKS::make_service_request() 
{
    this->service.request.pose_stamp.resize(1);
    this->service.request.pose_stamp[0] = this->endpoint.get_pose();
    this->service.request.seed_mode = 2;
} 

// GET IKS FUNCTION
// calls the service and gets the iks, or errors if there is none
void IKS::get_iks() 
{
    if (!this->client.call(this->service))
    {
        this->baxter.make_face(SAD);
        ROS_ERROR("Failure: unable to call service");
        exit(1);
    }

    if (!this->service.response.isValid[0]) 
    {
        this->baxter.make_face(SAD);
        ROS_ERROR("Failure: no valid ik joint solution");
        exit(1);
    }

    this->baxter.make_face(HAPPY);
    ROS_INFO("\tinverse kinematic solution found");
 	ROS_INFO("\t(%f, %f, %f)", 
	    this->endpoint.get_point().x, 
	    this->endpoint.get_point().y, 
	    this->endpoint.get_point().z);
    
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
