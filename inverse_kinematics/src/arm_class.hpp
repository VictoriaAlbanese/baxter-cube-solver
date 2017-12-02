//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: arm_class.hpp
//
// Description: Defines a class where positions and 
// properties of the joints of baxter's arms can be set 
// and manipulated
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
        
		// members
		bool arm_side;
        baxter_core_msgs::JointCommand orders;
        vector<double> current_joint_positions;
        ros::Publisher pub;
        ros::Subscriber sub;

		// functions
        void joint_callback(const sensor_msgs::JointStateConstPtr& msg);
        bool is_positioned();

    public:
        
		// members
		bool initialized;
        bool done;

		// functions
		Arm();
        Arm(ros::NodeHandle handle, bool arm_side); 
		void move_to(baxter_core_msgs::JointCommand new_order);
        void send_home(); 
};

#endif // ARM_CLASS_HPP

//////////////////////////////////////////////////////////////
