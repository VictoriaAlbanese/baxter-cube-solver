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
#include "baxter_core_msgs/EndpointState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"

#include "endpoint_class.hpp"
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
        geometry_msgs::Pose endpoint;
        ros::Publisher order_pub;
        ros::Publisher point_pub;
        ros::Subscriber joint_sub;
        ros::Subscriber point_sub;
        bool joints_initialized;
        bool point_initialized;
        bool ready_;
        bool done_;

        // functions
        void joint_callback(const sensor_msgs::JointStateConstPtr& msg);
        void point_callback(const baxter_core_msgs::EndpointStateConstPtr& msg);
        void init();
        void get_ready();
        bool is_positioned();

    public:

        // members
        Gripper gripper;

        // functions
        Arm();
        Arm(ros::NodeHandle handle, bool arm_side); 
        bool initialized() { return this->joints_initialized && this->point_initialized && this->gripper.initialized(); } 
        bool done() { return this->done_; }
        bool ready_for_pickup() { return (this->endpoint.position.z < -0.1); }
	    void move_to(baxter_core_msgs::JointCommand new_order);
        void adjust_endpoint_x(float offset);
        void adjust_endpoint_y(float offset);
        void lower_arm();
        void turn_wrist(float offset);
        void send_home(); 
};

#endif // ARM_CLASS_HPP

//////////////////////////////////////////////////////////////
