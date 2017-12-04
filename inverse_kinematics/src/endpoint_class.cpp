////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: ik_client_class.cpp
//
// Description: This implements a class which represents 
// the inverse kinematic solver client
//
//////////////////////////////////////////////////////////////

#include "endpoint_class.hpp"

Endpoint::Endpoint() 
{
    this->initialized = false;
}

Endpoint::Endpoint(ros::NodeHandle handle) 
{
    this->initialized = false;
    this->sub = handle.subscribe<geometry_msgs::Point>("highest_point", 10, &Endpoint::callback, this);

    while (!this->initialized) 
    {
        ros::spinOnce();
    }
}

void Endpoint::callback(const geometry_msgs::Point::ConstPtr& msg) 
{
    this->initialized = true;
    this->point = *msg;
    this->sub.shutdown();
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
