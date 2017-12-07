//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: gripper_class.hpp
//
// Description: Declares a class where properties and 
// position of the gripper can be retrieved and manipulated
//
//////////////////////////////////////////////////////////////

#ifndef GRIPPER_CLASS_HPP
#define GRIPPER_CLASS_HPP

#include "ros/ros.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/EndEffectorState.h"

#define LEFT 0
#define RIGHT 1

class Gripper
{
    private: 
        
        // members
        int id;
        bool arm_side;
        bool initialized;
        float position;
        ros::Subscriber subscriber;
        ros::Publisher publisher;

        // functions
        void callback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg);

    public:

        // members
        bool calibrated;
        bool ready;
        bool gripping;
       
        // functions
        Gripper();
        Gripper(ros::NodeHandle handle, bool arm_side); 
        int get_id() { return this->id; }
        void calibrate();
        void grip();
        void release();
};

#endif // GRIPPER_CLASS_HPP

//////////////////////////////////////////////////////////////
