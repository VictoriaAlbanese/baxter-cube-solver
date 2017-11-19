////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: iks_class.hpp
//
// Description: Declares a class which abstracts away 
// baxter's inverse kinematic solver
//
//////////////////////////////////////////////////////////////

#include "ros/ros.h"
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
#define YAW 0;

using std::string;

class IKS 
{
	private:
		Endpoint point;
		ros::ServiceClient client;
		baxter_core_msgs::SolvePositionIK service;
		baxter_core_msgs::JointCommand solved_state;

	public:
		IKS();
		IKS(ros::NodeHandle handle);
		sensor_msgs::JointState get_solved_state() { return this->solved_state; };
		void get_iks();
		void make_service_request(); 
		geometry_msgs::PoseStamped get_pose();
	
};

//////////////////////////////////////////////////////////////
