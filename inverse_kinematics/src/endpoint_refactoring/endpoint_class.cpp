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
// does the ros initialization 
Endpoint::Endpoint(ros::NodeHandle handle) 
{
    this->init();
    this->sub = handle.subscribe<geometry_msgs::Point>("goal_point", 10, &Endpoint::callback, this);
 
    ROS_INFO("position:");
    ROS_INFO("\tx:[%f]", this->pose.pose.position.x);
    ROS_INFO("\ty:[%f]", this->pose.pose.position.y);
    ROS_INFO("\tz:[%f]", this->pose.pose.position.z);
    ROS_INFO("orientation:");
    ROS_INFO("\tx:[%f]", this->pose.pose.orientation.x);
    ROS_INFO("\ty:[%f]", this->pose.pose.orientation.y);
    ROS_INFO("\tz:[%f]", this->pose.pose.orientation.z);

}

// SET POSITION FUNCTION
// set the position PART OF THE POSE OBJECT
void Endpoint::set_position(geometry_msgs::Point point, float z_plane) 
{
    geometry_msgs::Point new_point;
    new_point.x = point.x; 
    new_point.y = point.y;
    new_point.z = z_plane;

    this->pose.pose.position = new_point;
 
    ROS_INFO("position:");
    ROS_INFO("\tx:[%f]", this->pose.pose.position.x);
    ROS_INFO("\ty:[%f]", this->pose.pose.position.y);
    ROS_INFO("\tz:[%f]", this->pose.pose.position.z);
}

// SET ORIENTATION
// this function sets a quaternion that corresponds to an
// orientation where the grippers are pointing straight down
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

    this->pose.pose.orientation = q;

    ROS_INFO("orientation:");
    ROS_INFO("\tx:[%f]", this->pose.pose.orientation.x);
    ROS_INFO("\ty:[%f]", this->pose.pose.orientation.y);
    ROS_INFO("\tz:[%f]", this->pose.pose.orientation.z);
}

//////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CALLBACK FUNCTION
// sets the point to the value 
// published to the goal point topic
void Endpoint::callback(const geometry_msgs::Point::ConstPtr& msg) 
{
    this->initialized_ = true;
    
    this->initialize_pose();
    this->set_position(*msg);

    ROS_INFO("callback");
     
    ROS_INFO("position:");
    ROS_INFO("\tx:[%f]", this->pose.pose.position.x);
    ROS_INFO("\ty:[%f]", this->pose.pose.position.y);
    ROS_INFO("\tz:[%f]", this->pose.pose.position.z);
    ROS_INFO("orientation:");
    ROS_INFO("\tx:[%f]", this->pose.pose.orientation.x);
    ROS_INFO("\ty:[%f]", this->pose.pose.orientation.y);
    ROS_INFO("\tz:[%f]", this->pose.pose.orientation.z);
}

// INIT FUNCTION
// common code in constructors
// sets all bools to false and point to (-1, -1, -1)
void Endpoint::init() 
{
    this->initialized_ = false;
    this->initialize_pose();
}

// INITIALIZE POSE FUNCTION
// eets a default pose for the endpoint
void Endpoint::initialize_pose() 
{
    geometry_msgs::Pose pose;
    pose.position = this->initialize_position();
    pose.orientation = this->initialize_orientation();

    geometry_msgs::PoseStamped ps;
    ps.header = this->initialize_header();
    ps.pose = pose;

    this->pose = ps;
}

// INITIALIZE HEADER FUNCTION
// gets a default header for the pose stamped object
std_msgs::Header Endpoint::initialize_header() 
{
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.frame_id = "base";
    
    return header;
}

// INITIALIZE POSITION FUNCTION
// gets a default position for the pose 
geometry_msgs::Point Endpoint::initialize_position() 
{
    geometry_msgs::Point new_point;
    new_point.x = -1; 
    new_point.y = -1;
    new_point.z = -1;
      
    return new_point;
}

// INITIALIZE ORIENTATION
// this function returns a quaternion that corresponds to an
// orientation where the grippers are pointing straight down
geometry_msgs::Quaternion Endpoint::initialize_orientation() 
{
    double YAW = 0.0;

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
