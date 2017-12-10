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
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"

#define ROLL 0
#define PITCH 3.14

class Endpoint
{
    private: 

        // members
        bool initialized;
        geometry_msgs::PoseStamped endpoint;
        ros::Subscriber sub;

        // functions
        void callback(const geometry_msgs::Pose::ConstPtr& msg);
        void init();
        void initialize_pose();
        std_msgs::Header initialize_header();
        geometry_msgs::Point initialize_position();
        geometry_msgs::Quaternion initialize_orientation();

    public:

        // members
        // n/a

        // functions
        Endpoint();
        Endpoint(ros::NodeHandle handle); 
        geometry_msgs::PoseStamped get_pose() { return this->endpoint; }
        geometry_msgs::Point get_point() { return this->endpoint.pose.position; }
        void set_position(geometry_msgs::Point point, float z_plane = 0.10);
        void set_orientation(float YAW = 0.0);
};

#endif // ENDPOINT_CLASS_HPP

//////////////////////////////////////////////////////////////
