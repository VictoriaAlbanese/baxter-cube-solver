//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: endpoint_class.hpp
//
// Description: Declares a class which represents the 
// position aspect of the goal endpoint for the iks
//
//////////////////////////////////////////////////////////////

#ifndef ENDPOINT_CLASS_HPP
#define ENDPOINT_CLASS_HPP

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class Endpoint
{
    private: 

        // members
        bool initialized;
        ros::Subscriber sub;

        // functions
        void callback(const geometry_msgs::Point::ConstPtr& msg);

    public:

        // members
        geometry_msgs::Point point;
        
        // functions
        Endpoint();
        Endpoint(ros::NodeHandle handle); 
};

#endif // ENDPOINT_CLASS_HPP

//////////////////////////////////////////////////////////////
