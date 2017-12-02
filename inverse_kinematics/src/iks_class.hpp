//////////////////////////////////////////////////////////////
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
#include "sensor_msgs/JointState.h"

#include "endpoint_class.hpp"

#include <cstdlib>
#include <string>

#define TABLE_CENTER 0

using std::string;

class IKS 
{
    private:
        
		// members
		Endpoint point;
		ros::ServiceClient client;
		baxter_core_msgs::SolvePositionIK service;
        sensor_msgs::JointState solved_state;
        baxter_core_msgs::JointCommand orders;

		// functions
		void make_service_request(); 
		void get_iks();
        void iks_to_joint_command();

	public:

		// members (n/a)
		
		// functions
		IKS();
		IKS(ros::NodeHandle handle);
        sensor_msgs::JointState get_solved_state() { return this->solved_state; };
        baxter_core_msgs::JointCommand get_orders() { return this->orders; };
};

#endif // IKS_CLASS_HPP

//////////////////////////////////////////////////////////////
