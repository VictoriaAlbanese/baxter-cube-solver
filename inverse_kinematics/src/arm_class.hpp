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
#include "baxter_core_msgs/JointCommand.h"
#include "sensor_msgs/JointState.h"

#include <string>

#define LEFT 0
#define RIGHT 1

using std::string;
using std::vector;

class Arm
{
    private: 
        bool arm_side;
        baxter_core_msgs::JointCommand orders;
        vector<double> current_joint_positions;
        ros::Publisher pub;
        ros::Subscriber sub;

    public:
        Arm();
        Arm(ros::NodeHandle handle, bool arm_side); 
        void update_current_joint_positions(const sensor_msgs::JointStateConstPtr& msg);
        bool is_positioned();
        void send_home(); 
        bool is_done;
};

#endif // ARM_CLASS_HPP

//////////////////////////////////////////////////////////////
