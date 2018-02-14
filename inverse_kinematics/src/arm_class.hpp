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
#include "iks_class.hpp"

#include <string>

#define LEFT 0
#define RIGHT 1

#define CW 0
#define CCW 1

#define HOME 1
#define CENTER 2

#define X 1
#define Y 2
#define Z 3

using std::string;
using std::vector;

class Arm
{
    private: 
        
        // members
        bool arm_side;
        bool ready_;
        bool done_;
        
        bool joints_initialized;
        baxter_core_msgs::JointCommand orders;
        vector<double> current_joint_positions;
        ros::Publisher order_pub;
        ros::Subscriber joint_sub;

        bool point_initialized;
        geometry_msgs::Pose endpoint;
        ros::Publisher point_pub;
        ros::Subscriber point_sub;
              
        // functions
        void joint_callback(const sensor_msgs::JointStateConstPtr& msg);
        void point_callback(const baxter_core_msgs::EndpointStateConstPtr& msg);
        void init();
        void get_ready();
        bool is_positioned();
        void send_home(); 
        void bring_center(); 

    public:

        // members
        Gripper gripper;

        // functions
        Arm();
        Arm(ros::NodeHandle handle, bool arm_side); 
        bool initialized() { return this->joints_initialized && this->point_initialized && this->gripper.initialized(); } 
        bool done() { return this->done_; }
        bool ready_for_pickup() { return (this->endpoint.position.z < -0.1); }
        float get_endpoint_x() { return this->endpoint.position.x; };
        float get_endpoint_y() { return this->endpoint.position.y; };
        float get_endpoint_z() { return this->endpoint.position.z; };
      	
        void move_to(int hardcoded_state);
        void move_to(baxter_core_msgs::JointCommand new_order);
        void turn_wrist_to(float new_position, bool is_increment = false);
       
        void adjust_endpoint(int direction, float new_position, bool is_increment = false);
        void lower_arm();
        void make_endpoints_perpendicular();
};

#endif // ARM_CLASS_HPP

//////////////////////////////////////////////////////////////
