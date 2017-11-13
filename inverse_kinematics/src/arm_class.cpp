////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Date: October 11, 2017
// Filename: arm_class.cpp
//
// Description: This creates a class where properties of the 
// arm can be set by the arm subscriber's callback 
//
//////////////////////////////////////////////////////////////

#include "arm_class.hpp"

Arm::Arm() 
{
    this->arm_side = RIGHT;
    this->is_enabled = false;
}

Arm::Arm(ros::NodeHandle handle, bool arm_side)
{
    string pub_topic, sub_topic;
    if (arm_side == LEFT) 
    {
        pub_topic = "/robot/limb/left/joint_command";
        sub_topic = "/robot/limb/left/state";
    }

    else 
    {
        pub_topic = "/robot/limb/right/joint_command";
        sub_topic = "/robot/limb/right/state";
    }
    
    this->arm_side = arm_side; 
    this->is_enabled = false;
    this->pub_enabled = handle.advertise<std_msgs::Bool>("/robot/set_super_enable", 10);
    this->sub_enabled = handle.subscribe<baxter_core_msgs::AssemblyState>(sub_topic, 10, &Arm::state_cb, this);
    this->pub = handle.advertise<baxter_core_msgs::JointCommand>(pub_topic, 10);
}

void Arm::state_cb(const baxter_core_msgs::AssemblyState::ConstPtr& msg) 
{
    this->is_enabled = msg->enabled;
}

void Arm::toggle_enable() 
{
    std_msgs::Bool toggle;

    //if (!this->is_enabled) toggle.data = true;
    //else toggle.data = false;

    toggle.data = true;
    this->pub_enabled.publish(toggle);
}

void Arm::send_home(bool state) 
{
    if (state == SETUP && !this->is_enabled) this->toggle_enable();

    baxter_core_msgs::JointCommand msg;
    msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

    if (this->arm_side == LEFT) 
    {
        msg.names.push_back("left_s0");
        msg.names.push_back("left_s1");
        msg.names.push_back("left_e0");
        msg.names.push_back("left_e1");
        msg.names.push_back("left_w0");
        msg.names.push_back("left_w1");
        msg.names.push_back("left_w2");

        msg.command.resize(msg.names.size());
        msg.command[0] = 0.141;
        msg.command[1] = 1.998;
        msg.command[2] = 0.361;
        msg.command[3] = -1,366;
        msg.command[4] = 0.0;
        msg.command[5] = 0.938;
        msg.command[6] = 1.308;
    }

    else
    {
        msg.names.push_back("right_s0");
        msg.names.push_back("right_s1");
        msg.names.push_back("right_e0");
        msg.names.push_back("right_e1");
        msg.names.push_back("right_w0");
        msg.names.push_back("right_w1");
        msg.names.push_back("right_w2");

        msg.command.resize(msg.names.size());
        msg.command[0] = -0.141;
        msg.command[1] = 1.998;
        msg.command[2] = -0.361;
        msg.command[3] = -1,366;
        msg.command[4] = 0.0;
        msg.command[5] = 0.938;
        msg.command[6] = -1.308;
    }

    this->pub.publish(msg);
    
    if (state == CLEANUP && this->is_enabled) this->toggle_enable();
}

//////////////////////////////////////////////////////////////
