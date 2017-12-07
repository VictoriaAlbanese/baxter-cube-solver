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
// does not initialize ros; initializes the 
// endpoint istelf to (-1, -1, -1), an error state
Endpoint::Endpoint() 
{
    this->initialized = false;

    geometry_msgs::Point p;
    p.x = -1;
    p.y = -1;
    p.z = -1;
    this->point = p;
}

// CONSTRUCTOR
// does the ros initialization; waits for 
// the callback to properly initialize the point
Endpoint::Endpoint(ros::NodeHandle handle) 
{
    this->initialized = false;
    this->sub = handle.subscribe<geometry_msgs::Point>("goal_point", 10, &Endpoint::callback, this);

    while (!this->initialized) ros::spinOnce();
}

//////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CALLBACK FUNCTION
// sets the point to the value 
// published to the goal point topic
void Endpoint::callback(const geometry_msgs::Point::ConstPtr& msg) 
{
    this->initialized = true;
    this->point = *msg;
    
    /*
    ROS_INFO("Point initialized");
    if (this->initialized == true) ROS_INFO("\tinitialized: true");
    else ROS_INFO("\tinitialized: false");
    ROS_INFO("\tx[%f]", this->point.x);
    ROS_INFO("\ty[%f]", this->point.y);
    ROS_INFO("\tz[%f]", this->point.z);
    */
}

//////////////////////////////////////////////////////////////
