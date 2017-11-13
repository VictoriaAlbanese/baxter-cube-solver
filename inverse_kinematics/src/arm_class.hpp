////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: arm_class.hpp
//
// Description: This declares a class which represents 
// Baxter's left and right arms 
//
//////////////////////////////////////////////////////////////

#ifndef ARM_CLASS_HPP
#define ARM_CLASS_HPP

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "baxter_core_msgs/AssemblyState.h"
#include "baxter_core_msgs/JointCommand.h"

#include <string>

#define LEFT 0
#define RIGHT 1
#define CLEANUP 0
#define SETUP 1

using std::string;

class Arm
{
    private: 
        bool arm_side;
        bool is_enabled;
        ros::Publisher pub_enabled;
        ros::Subscriber sub_enabled;
        ros::Publisher pub;

    public:
        Arm();
        Arm(ros::NodeHandle handle, bool arm_side); 
        void state_cb(const baxter_core_msgs::AssemblyState::ConstPtr& msg);
        void toggle_enable(); 
        void send_home(bool state); 
};

#endif // ARM_CLASS_HPP

//////////////////////////////////////////////////////////////
