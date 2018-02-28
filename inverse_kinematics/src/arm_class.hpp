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

#include <ros/ros.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

#include "endpoint_class.hpp"
#include "gripper_class.hpp"
#include "iks_class.hpp"
#include "universal_constants.hpp"

#define R_AWAY 0.0
#define R_SOFT_HOLD 0.03
#define R_FIRM_HOLD 0.05
#define L_AWAY 0.0
#define L_SOFT_HOLD (-0.01)
#define L_FIRM_HOLD (-0.03)

#define CW2 3.0
#define CW 1.6
#define UP 0.0
#define CCW (-1.6)
#define CCW2 (-3.0)

#define HOME 1
#define CENTER 2
#define P_CENTER 3
#define ENDPOINT 4
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
        geometry_msgs::Point current_track;

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
        geometry_msgs::Pose center_perpendicularly(); 
        geometry_msgs::Point set_p(float new_x, float new_y, float new_z);
        geometry_msgs::Quaternion set_q(float new_x, float new_y, float new_z, float new_w);

    public:

        // members
        Gripper gripper;
        IKS iks;

        // functions
        Arm();
        Arm(ros::NodeHandle handle, bool arm_side); 
        bool initialized() { return this->joints_initialized && this->point_initialized && this->gripper.initialized(); } 
        bool done() { return this->done_; }
        bool ready_for_pickup() { return (this->endpoint.position.z < -0.1); }
        bool side() { return this->arm_side; };

        geometry_msgs::Pose get_endpoint() { return this->endpoint; }; 
        float get_endpoint_x() { return this->endpoint.position.x; };
        float get_endpoint_y() { return this->endpoint.position.y; };
        float get_endpoint_z() { return this->endpoint.position.z; };
      	
        bool move_to(int hardcoded_state);
        void move_to(baxter_core_msgs::JointCommand new_order);
        void turn_wrist_to(float new_position, bool is_increment = false);
       
        void set_endpoint(int hardcoded_state);
        void set_endpoint(geometry_msgs::Point point, geometry_msgs::Quaternion quaternion);
        void adjust_endpoint(int direction, float new_position, bool is_increment = false);
};

#endif // ARM_CLASS_HPP

//////////////////////////////////////////////////////////////
