////////////////////////////////////////////////////i//////////
//
// Programmer: Victoria Albanese
// Filename: gripper_class.cpp
//
// Description: Implements a class where properties and 
// position of the gripper can be retrieved and manipulated
//
//////////////////////////////////////////////////////////////

#include "gripper_class.hpp"

//////////////////////////////////////////////////////////////

// Public Members

// DEFAULT CONSTRUCTOR
// does not do ros initialization
// makes a right gripper by default
Gripper::Gripper()
{
    this->init();
    this->arm_side = RIGHT;
}

// CONSTRUCTOR
// same as default but also
// does the ros initialization
Gripper::Gripper(ros::NodeHandle handle, bool arm_side)
{
    this->init();
    this->arm_side = arm_side;

    string sub_topic;
    if (this->arm_side == LEFT) sub_topic = "robot/end_effector/left_gripper/state";
    else sub_topic = "robot/end_effector/right_gripper/state";

    string pub_topic;
    if (this->arm_side == LEFT) pub_topic = "robot/end_effector/left_gripper/command";
    else pub_topic = "robot/end_effector/right_gripper/command";

    this->subscriber = handle.subscribe(sub_topic, 10, &Gripper::callback, this);
    this->publisher = handle.advertise<baxter_core_msgs::EndEffectorCommand>(pub_topic, 10);
}

// CALIBRATE FUNCTION
// calibrates the gripper
void Gripper::calibrate() 
{
    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "calibrate";
    this->publisher.publish(msg);
}

// GRIP FUNCTION
// closes the grippers to grasp an object
void Gripper::grip() 
{
    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "grip";
    this->publisher.publish(msg);
}

// RELEASE FUNCTION
// opens the grippers to release an object
void Gripper::release() 
{
    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "release";
    this->publisher.publish(msg);
}

//////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CALLBACK FUNCTION
// initializes gripper fields
void Gripper::callback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg) 
{
    this->initialized_ = true;

    this->calibrated_ = msg->calibrated;
    this->ready_ = msg->ready;
    this->gripping_ = msg->gripping;
    
    this->id = msg->id;
    this->position = msg->position;
}

// INIT FUNCTION
// common code in constructors
// sets all bools to false, ints to -1
void Gripper::init() 
{
    this->initialized_ = false;
    
    this->calibrated_ = false;
    this->ready_ = false;
    this->gripping_ = false;

    this->id = -1;
    this->position = -1;
}

//////////////////////////////////////////////////////////////
