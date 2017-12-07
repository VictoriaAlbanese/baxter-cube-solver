////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: arm_class.hpp
//
// Description: Declares a class where positions of baxter's
// arm joints can be manipulated 
//
//////////////////////////////////////////////////////////////

#ifndef ARM_CLASS_HPP
#define ARM_CLASS_HPP

#include "ros/ros.h"
#include "baxter_core_msgs/JointCommand.h"
#include "sensor_msgs/JointState.h"
#include "gripper_class.hpp"

#include <string>

#define LEFT 0
#define RIGHT 1

using std::string;
using std::vector;

class Arm
{
    private: 
        
        // members
        bool arm_side;
        baxter_core_msgs::JointCommand orders;
        vector<double> current_joint_positions;
        ros::Publisher pub;
        ros::Subscriber sub;
        bool initialized;
        bool ready;

        // functions
        void joint_callback(const sensor_msgs::JointStateConstPtr& msg);
        void execute_orders(baxter_core_msgs::JointCommand new_orders);
        void get_ready();
        bool is_positioned();

    public:

        // members
        bool done;
        Gripper gripper;

        // functions
        Arm();
        Arm(ros::NodeHandle handle, bool arm_side); 
	    void move_to(baxter_core_msgs::JointCommand new_order);
        void turn_wrist(float offset);
        void send_home(); 
};

#endif // ARM_CLASS_HPP

//////////////////////////////////////////////////////////////
