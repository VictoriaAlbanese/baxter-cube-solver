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
Endpoint::Endpoint(ros::NodeHandle handle, bool arm_side) 
{
    this->init();

    string sub_topic;
    if (arm_side == LEFT) sub_topic = "/left/goal_point";
    else sub_topic = "/right/goal_point";
    this->sub = handle.subscribe<geometry_msgs::Pose>(sub_topic, 10, &Endpoint::callback, this);
}

//////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CALLBACK FUNCTION
// sets the point to the value 
// published to the goal point topic
void Endpoint::callback(const geometry_msgs::Pose::ConstPtr& msg) 
{
    this->initialize_pose();
    this->endpoint.pose = *msg;
    this->initialized_ = true;
}

// INIT FUNCTION
// common code in constructors
// sets all bools to false
void Endpoint::init() 
{
    this->initialized_ = false;
    this->initialize_pose();
}

// INITIALIZE POSE FUNCTION
// initialize the endpoint
void Endpoint::initialize_pose() 
{
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.frame_id = "base";

    geometry_msgs::Pose pose;
    pose.position.x = -1.0;
    pose.position.y = -1.0;
    pose.position.z = -1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 1.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    geometry_msgs::PoseStamped ps;
    ps.header = header;
    ps.pose = pose;

    this->endpoint = ps;
}

//////////////////////////////////////////////////////////////
