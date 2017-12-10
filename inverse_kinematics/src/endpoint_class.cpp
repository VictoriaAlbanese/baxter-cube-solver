//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: endpoint_class.cpp
//
// Description: Implements a class which represents the 
// position aspect of the goal endpoint for the iks
//
//////////////////////////////////////////////////////////////

#include "endpoint_class.hpp"

//////////////////////////////////////////////////////////////

// Public Members

// DEFAULT CONSTRUCTOR
// does not initialize ros 
Endpoint::Endpoint() 
{
    this->init();
}

// CONSTRUCTOR
// does the ros initialization; waits for 
// the callback to properly initialize the point
Endpoint::Endpoint(ros::NodeHandle handle) 
{
    this->init();
    this->sub = handle.subscribe<geometry_msgs::Point>("goal_point", 10, &Endpoint::callback, this);

    while (!this->initialized) ros::spinOnce();
}

// SET POSITION FUNCTION
// set the position aspect of the pose
void Endpoint::set_position(geometry_msgs::Point point, float z_plane) 
{
    geometry_msgs::Point new_point;
    new_point.x = point.x; 
    new_point.y = point.y;
    new_point.z = z_plane;
      
    this->endpoint.pose.position = new_point;
}

// SET ORIENTATION
// sets the orientation aspect of the pose
// such that the grippers are pointing straight down
void Endpoint::set_orientation(float YAW) 
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

    this->endpoint.pose.orientation = q;
}

//////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CALLBACK FUNCTION
// sets the point to the value 
// published to the goal point topic
void Endpoint::callback(const geometry_msgs::Point::ConstPtr& msg) 
{
    this->initialized = true;
    this->initialize_pose();
    this->set_position(*msg);
    
    /*
    ROS_INFO("Point initialized");
    if (this->initialized == true) ROS_INFO("\tinitialized: true");
    else ROS_INFO("\tinitialized: false");
    ROS_INFO("\tx[%f]", this->point.x);
    ROS_INFO("\ty[%f]", this->point.y);
    ROS_INFO("\tz[%f]", this->point.z);
    */
}

// INIT FUNCTION
// common code in constructors
// sets all bools to false
void Endpoint::init() 
{
    this->initialized = false;
    this->initialize_pose();
}

// INITIALIZE POSE FUNCTION
// initialize the endpoint
void Endpoint::initialize_pose() 
{
    geometry_msgs::Pose pose;
    pose.position = this->initialize_position();
    pose.orientation = this->initialize_orientation();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = this->initialize_header();
    pose_stamped.pose = pose;

    this->endpoint = pose_stamped;
}

// INITIALIZE HEADER FUNCTION
// initialize the header of the pose stamped object
std_msgs::Header Endpoint::initialize_header() 
{
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.frame_id = "base";
        
    return header;
}

// INITIALIZE POSITION FUNCTION
// initialize the position aspect of the pose
geometry_msgs::Point Endpoint::initialize_position() 
{
    geometry_msgs::Point new_point;
    new_point.x = -1; 
    new_point.y = -1;
    new_point.z = -1;
      
    return new_point;
}

// INITIALIZE ORIENTATION
// initialize the orientation aspect of the pose
// such that the grippers are pointing straight down
geometry_msgs::Quaternion Endpoint::initialize_orientation() 
{ 
    float YAW = 0.0;

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
