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

    string state_sub_topic;
    if (this->arm_side == LEFT) state_sub_topic = "robot/end_effector/left_gripper/state";
    else state_sub_topic = "robot/end_effector/right_gripper/state";

    string ir_sub_topic;
    if (this->arm_side == LEFT) ir_sub_topic = "robot/range/left_hand_range/state";
    else ir_sub_topic = "robot/range/right_hand_range/state";

    string pub_topic;
    if (this->arm_side == LEFT) pub_topic = "robot/end_effector/left_gripper/command";
    else pub_topic = "robot/end_effector/right_gripper/command";

    this->state_sub = handle.subscribe(state_sub_topic, 1, &Gripper::state_callback, this);
    this->ir_sub = handle.subscribe(ir_sub_topic, 1, &Gripper::ir_callback, this);
    this->publisher = handle.advertise<baxter_core_msgs::EndEffectorCommand>(pub_topic, 1);
}

// CALIBRATE FUNCTION
// calibrates the gripper
void Gripper::calibrate() 
{
    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "calibrate";
    this->publisher.publish(msg);
    ros::Duration(1.0).sleep();
}

// GRIP FUNCTION
// closes the grippers to grasp an object
void Gripper::grip() 
{
    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "grip";
    this->publisher.publish(msg);
    ros::Duration(1.0).sleep();
}

// RELEASE FUNCTION
// opens the grippers to release an object
void Gripper::release() 
{
    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "release";
    this->publisher.publish(msg);
    ros::Duration(1.0).sleep();
}

//////////////////////////////////////////////////////////////

// Private Members & Callbacks

// STATE CALLBACK FUNCTION
// initializes gripper state fields
void Gripper::state_callback(const baxter_core_msgs::EndEffectorState::ConstPtr& msg) 
{
    this->state_initialized = true;

    this->calibrated_ = msg->calibrated;
    this->ready_ = msg->ready;
    this->gripping_ = msg->gripping;
    this->id = msg->id;
}

// IR CALLBACK FUNCTION
// initializes ir sensor range field
void Gripper::ir_callback(const sensor_msgs::Range& msg) 
{
    this->ir_initialized = true;
    this->range = msg.range;
}

// INIT FUNCTION
// common code in constructors
// sets all bools to false, ints to -1
void Gripper::init() 
{
    this->state_initialized = false;
    this->ir_initialized = false;
    
    this->calibrated_ = false;
    this->ready_ = false;
    this->gripping_ = false;
    
    this->id = -1;
    this->range = -1;
}

//////////////////////////////////////////////////////////////
