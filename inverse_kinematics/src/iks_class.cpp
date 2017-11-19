////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: iks_class.cpp
//
// Description: Declares a class which abstracts away 
// baxter's inverse kinematic solver
//
//////////////////////////////////////////////////////////////

#include "iks_class.hpp"

// DEFAULT CONSTRUCTOR
// Sets up a default endpoint
IKS::IKS() 
{
    this->arm_side = LEFT;
    this->point = Endpoint();
}

// ONE ARGUMENT CONSTRUCTOR
// Wait for an endpoint and then get the iks
IKS::IKS(ros::NodeHandle handle, bool arm_side) 
{
    this->arm_side = arm_side;
    string service_name;
    if (this->arm_side == LEFT) service_name = "ExternalTools/left/PositionKinematicsNode/IKService";
    else service_name = "ExternalTools/right/PositionKinematicsNode/IKService";

    this->point = Endpoint(handle);
	this->client = handle.serviceClient<baxter_core_msgs::SolvePositionIK>(service_name);
	
	this->make_service_request();
	this->get_iks();
    this->iks_to_joint_command();
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

// MAKE SERVICE REQUEST
// Makes the service request by setting up the endpoint
void IKS::make_service_request() 
{
    // Get the pose setup
    geometry_msgs::PoseStamped pose = get_pose();
	ROS_INFO("(%f, %f, %f)", 
	this->point.get_point().x, 
	this->point.get_point().y, 
	this->point.get_point().z);

    // Make the service request
    this->service.request.pose_stamp.resize(1);
    this->service.request.pose_stamp[0] = pose;
    this->service.request.seed_mode = 2;
} 

// GET DESIRED POSE
// Make the stamped_pose object
geometry_msgs::PoseStamped IKS::get_pose() 
{
    // Make a header for the pose_stamped
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.frame_id = "base";
        
    // Make the point for the pose
    // Keep z at 0 plane always, fine tune later
    geometry_msgs::Point new_point;
    new_point.x = this->point.get_point().x; 
    new_point.y = this->point.get_point().y;
    new_point.z = 0.1;
      
    // Make the pose for the pose_stamped
    geometry_msgs::Pose pose;
    pose.position = new_point;
    pose.orientation = get_orientation();

    // Make the stamped pose
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = pose;

    return pose_stamped;
}

// GET ORIENTATION
// This function returns a quaternion that corresponds to an 
// orientation where the grippers pointing straight down
geometry_msgs::Quaternion IKS::get_orientation() 
{ 
    double mathc1 = cos(PITCH);
    double maths1 = sin(PITCH);
    double mathc2 = cos(YAW);
    double maths2 = sin(YAW);
    double mathc3 = cos(ROLL);
    double maths3 = sin(ROLL);
                                                                     
    double oriw = sqrt(1.0 + mathc1 * mathc2 + mathc1 * mathc3 - maths1 * maths2 * maths3 + mathc2 * mathc3) / 2.0;
    double oriw4 = (4.0 * oriw);
        
    double orix = (mathc2 * maths3 + mathc1 * maths3 + maths1 * maths2 * mathc3) / oriw4;
    double oriy = (maths1 * mathc2 + maths1 * mathc3 + mathc1 * maths2 * maths3) / oriw4;
    double oriz = (-maths1 * maths3 + mathc1 * maths2 * mathc3 + maths2) / oriw4;

	geometry_msgs::Quaternion q;
    q.x = orix;
    q.y = oriy;
    q.z = oriz;
    q.w = oriw;

    return q;    
}

//////////////////////////////////////////////////////////////
