//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: endpoint_class.cpp
//
// Description: Implements a class which represents the
// the desired and current endpoints for the iks
//
//////////////////////////////////////////////////////////////

#include "endpoint_class.hpp"

//////////////////////////////////////////////////////////////

// Public Member Functions

// DEFAULT CONSTRUCTOR
// is not yet initialized, no ros setup
Endpoint::Endpoint() 
{
    this->initialized = false;
}

// CONSTRUCTOR
// not yet initialized, does the ros setup
// waits for the node to be initialized (get the desired endpoint)
Endpoint::Endpoint(ros::NodeHandle handle) 
{
    this->initialized = false;
    this->sub = handle.subscribe<geometry_msgs::Point>("highest_point", 10, &Endpoint::callback, this);
	
	this->wait_for_desired_point();
}

// WAIT FOR DESIRED POINT FUNCTION
// spins until the point is initialized (since the 
// pcl node takes a long time, it is necessary to wait)
void Endpoint::wait_for_desired_point() 
{
    while (!this->initialized) 
    {
        ros::spinOnce();
    }
}

//////////////////////////////////////////////////////////////

// Private Helper Functions & Callbacks

// CALLBACK FUNCTION
// this finally initializes the endpoint, setting its 
// pose and orinetation, then closing the subscriber
void Endpoint::callback(const geometry_msgs::Point::ConstPtr& msg) 
{
    this->get_point(*msg);
    this->sub.shutdown();
    
	this->initialized = true;
}

// GET DESIRED POSE
// Make the default stamped_pose object
void Endpoint::get_point(geometry_msgs::Point point) 
{
    // Make a header for the pose_stamped
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.frame_id = "base";
        
    // Make the point for the pose
    // Keep z at 0 plane always, fine tune later
    geometry_msgs::Point new_point;
    new_point.x = point.x; 
    new_point.y = point.y;
    new_point.z = 0.10;
      
    // Make the pose for the pose_stamped
    geometry_msgs::Pose pose;
    pose.position = new_point;
    pose.orientation = get_orientation();

    // Make the stamped pose
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = pose;

    this->point = pose_stamped;
}

// GET ORIENTATION
// This function returns a quaternion that corresponds to an 
// orientation where the gripper is pointing straight down
geometry_msgs::Quaternion Endpoint::get_orientation() 
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
