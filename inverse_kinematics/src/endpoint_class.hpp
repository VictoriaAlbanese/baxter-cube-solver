////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: ik_client_class.hpp
//
// Description: This declares a class which represents 
// an endpoint for the ik solver
//
//////////////////////////////////////////////////////////////

#ifndef ENDPOINT_CLASS_HPP
#define ENDPOINT_CLASS_HPP

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class Endpoint
{
    private: 
        bool initialized;
        geometry_msgs::Point point;
        ros::Subscriber sub;

    public:
        Endpoint();
        Endpoint(ros::NodeHandle handle); 
        bool is_initialized() { return this->initialized; }
        geometry_msgs::Point get_point() { return this->point; }
        void callback(const geometry_msgs::Point::ConstPtr& msg);
};

#endif // ENDPOINT_CLASS_HPP

//////////////////////////////////////////////////////////////
