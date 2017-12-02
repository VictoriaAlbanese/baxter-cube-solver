////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: iks_class.hpp
//
// Description: Declares a class which abstracts away 
// baxter's inverse kinematic solver
//
//////////////////////////////////////////////////////////////

#ifndef IKS_CLASS_HPP
#define IKS_CLASS_HPP

#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/SolvePositionIK.h"
#include "baxter_core_msgs/SolvePositionIKRequest.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"

#include "endpoint_class.hpp"

#include <cstdlib>
#include <string>

#define ROLL 0
#define PITCH 3.14
#define YAW 0

#define LEFT 0
#define RIGHT 1

using std::string;

class IKS 
{
    private:
	    bool arm_side;
        Endpoint point;
		ros::ServiceClient client;
		baxter_core_msgs::SolvePositionIK service;
        sensor_msgs::JointState solved_state;
        baxter_core_msgs::JointCommand orders;

	public:
		IKS();
		IKS(ros::NodeHandle handle, bool arm_side);
        sensor_msgs::JointState get_solved_state() { return this->solved_state; };
        baxter_core_msgs::JointCommand get_orders();
        void iks_to_joint_command();
		void get_iks();
		void make_service_request(); 
		geometry_msgs::PoseStamped get_pose();
        geometry_msgs::Quaternion get_orientation();
};

#endif // IKS_CLASS_HPP

//////////////////////////////////////////////////////////////
