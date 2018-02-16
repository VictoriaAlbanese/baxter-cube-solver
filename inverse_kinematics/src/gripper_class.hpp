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
#include "sensor_msgs/Range.h"

#include <string>

#define LEFT 0
#define RIGHT 1

using std::string;

class Gripper
{
    private: 
        
        // members
        int id;
        bool arm_side;
        bool state_initialized;
        bool ir_initialized;
        bool calibrated_;
        bool ready_;
        bool gripping_;
        float range;
        ros::Subscriber state_sub;
        ros::Subscriber ir_sub;
        ros::Publisher publisher;

        // functions
        void state_callback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg);
        void ir_callback(const sensor_msgs::Range& msg);
        void init();

    public:

        // functions
        Gripper();
        Gripper(ros::NodeHandle handle, bool arm_side); 
        bool initialized() { return (this->state_initialized && this->ir_initialized); };
        bool calibrated() { return this->calibrated_; }
        bool ready() { return this->ready_; }
        bool gripping() { return this->gripping_; }
        float get_range() { return this->range; }
        void calibrate();
        void grip();
        void release();
};

#endif // GRIPPER_CLASS_HPP

//////////////////////////////////////////////////////////////
