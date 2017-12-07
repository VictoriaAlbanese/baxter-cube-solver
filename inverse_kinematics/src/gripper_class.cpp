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
// does not do ros initialization; sets bools to 
// default values of false, ints to -1, an error state
// makes a right gripper by default
Gripper::Gripper()
{
    this->initialized = false;

    this->calibrated = false;
    this->ready = false;
    this->gripping = false;
   
    this->arm_side = RIGHT;
    
    this->id = -1;
    this->position = -1;
}

// CONSTRUCTOR
// does the ros initialization
// waits for the callback to initialize other fields
Gripper::Gripper(ros::NodeHandle handle, bool arm_side)
{
    this->initialized = false;

    this->arm_side = arm_side;

    if (this->arm_side == LEFT) 
    {
        this->subscriber = handle.subscribe("robot/end_effector/left_gripper/state", 10, &Gripper::callback, this);
        this->publisher = handle.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command", 10);
    }
    
    else  
    {
        this->subscriber = handle.subscribe("robot/end_effector/right_gripper/state", 10, &Gripper::callback, this);
        this->publisher = handle.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/right_gripper/command", 10);
    }

    while (!this->initialized) ros::spinOnce();
}

// CALIBRATE FUNCTION
// calibrates the gripper
void Gripper::calibrate() 
{
    ROS_INFO("\tcalibrating %s gripper...", (arm_side == LEFT ? "left" : "right"));

    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "calibrate";
    this->publisher.publish(msg);

    while (!this->calibrated) 
    {
        ros::spinOnce();
        ROS_INFO("calibrating...");
    }
}

// GRIP FUNCTIONS
// publishes a message which causes the grippers
// to close and attempt to grasp an object
void Gripper::grip() 
{
    baxter_core_msgs::EndEffectorCommand msg;
    msg.id = this->id;
    msg.command = "grip";
    this->publisher.publish(msg);
}

// RELEASE FUNCTION
// publishes a message which causes the 
// grippers to open and release an object
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
    this->initialized = true;

    this->id = msg->id;
    this->calibrated = msg->calibrated;
    this->ready = msg->ready;
    this->gripping = msg->gripping;
    this->position = msg->position;
}

//////////////////////////////////////////////////////////////
