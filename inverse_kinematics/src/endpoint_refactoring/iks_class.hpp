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
#include "std_msgs/Bool.h"

#include "endpoint_class.hpp"
#include "face_display_class.hpp"

#include <cstdlib>
#include <string>

#define LEFT 0
#define RIGHT 1

using std::string;

class IKS 
{
    private:

        // members
        bool initialized_;
        bool arm_side;
        FaceDisplay baxter;
        Endpoint endpoint;
        ros::NodeHandle handle;
        ros::Publisher kill_pub;
        ros::ServiceClient client;
		baxter_core_msgs::SolvePositionIK service;
        sensor_msgs::JointState solved_state;
        baxter_core_msgs::JointCommand orders;

        // functions
        void make_service_request(); 
		void get_iks();
        void iks_to_joint_command();

	public:

        // functions
		IKS();
		IKS(ros::NodeHandle handle, bool arm_side);
        bool initialized() 
        { 
            if (this->initialized_) ROS_INFO("ik_solver initialized");
            else ROS_INFO("ik_solver NOT initialized");

            return this->initialized_; 
        }
        baxter_core_msgs::JointCommand get_orders();
        void find_cube(); 
		void kill_cloud();
};

#endif // IKS_CLASS_HPP

//////////////////////////////////////////////////////////////
