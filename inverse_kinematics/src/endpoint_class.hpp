//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: endpoint_class.hpp
//
// Description: Declares a class which represents the 
// desired endpoint for the iks
//
//////////////////////////////////////////////////////////////

#ifndef ENDPOINT_CLASS_HPP
#define ENDPOINT_CLASS_HPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"

#define ROLL 0
#define PITCH 3.14
#define YAW 0

class Endpoint
{
    private:

		// members 
        ros::Subscriber sub;

		// functions
        void callback(const geometry_msgs::Point::ConstPtr& msg);
		void set_pose();
        geometry_msgs::Quaternion set_orientation();

    public:

		// members
        bool initialized;
        geometry_msgs::PoseStamped point;
		
		// functions
        Endpoint();
        Endpoint(ros::NodeHandle handle); 
		void wait_for_desired_point();	
};

#endif // ENDPOINT_CLASS_HPP

//////////////////////////////////////////////////////////////
