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

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>
#include <string>

#include "universal_constants.hpp"

using std::string;

class Endpoint
{
    private: 

        // members
        bool initialized_;
        geometry_msgs::PoseStamped endpoint;
        ros::Subscriber sub;

        // functions
        void callback(const geometry_msgs::Pose::ConstPtr& msg);
        void init();
        void initialize_pose();

    public:

        // functions
        Endpoint();
        Endpoint(ros::NodeHandle handle, bool arm_side); 
        bool initialized() { return this->initialized_; }
        void uninitialize() { this->initialized_ = false; };
        geometry_msgs::PoseStamped get_pose() { return this->endpoint; }
        geometry_msgs::Point get_point() { return this->endpoint.pose.position; }
        geometry_msgs::Quaternion get_q() { return this->endpoint.pose.orientation; }
};

#endif // ENDPOINT_CLASS_HPP

//////////////////////////////////////////////////////////////
