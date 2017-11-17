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
    this->is_done = false;
}

Arm::Arm(ros::NodeHandle handle, bool arm_side)
{
    string pub_topic;
    if (arm_side == LEFT) pub_topic = "/robot/limb/left/joint_command";
    else pub_topic = "/robot/limb/right/joint_command";
    
    this->arm_side = arm_side; 
    this->is_done = false;
    this->pub = handle.advertise<baxter_core_msgs::JointCommand>(pub_topic, 10);
    this->sub = handle.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Arm::update_current_joint_positions, this);
}

void Arm::update_current_joint_positions(const sensor_msgs::JointStateConstPtr& msg) 
{
    // clear the current vector
    this->current_joint_positions.clear();
    
    // update it with the new arm positions
    for (size_t i = 0; i < msg->position.size(); i++)
    {
        if (msg->name[i].find((arm_side == LEFT ? "left" : "right")) != std::string::npos)
        {   
            this->current_joint_positions.push_back(msg->position[i]);
        }   
    }

    // if the robot is in the correct position, 
    // we can stop publishing commands to move
    // otherwise, keep on publishing
    if (this->is_positioned()) this->is_done = true;
    else this->pub.publish(this->orders);
}

bool Arm::is_positioned() 
{
    for(size_t i = 0; i < this->orders.command.size(); i++)
    {
        ROS_INFO("moving %s from [%f] to [%f]", orders.names[i].c_str(), this->current_joint_positions[i], this->orders.command[i]);

        if(fabs(this->orders.command[i] - this->current_joint_positions[i]) > 0.03) {
            return false;
        }
    }

    return true;
}

void Arm::send_home() 
{
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
        msg.command[0] = 0.0; 
        msg.command[1] = -0.8;
        msg.command[2] = 0.0;
        msg.command[3] = 1.3;
        msg.command[4] = 0.0;
        msg.command[5] = 0.95; 
        msg.command[6] = 0.0;
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
        msg.command[0] = 0.0;
        msg.command[1] = -0.8;
        msg.command[2] = 0.0; 
        msg.command[3] = 1.3;
        msg.command[4] = 0.0; 
        msg.command[5] = 0.95;
        msg.command[6] = 0.0; 
    }

    this->orders = msg;
    //this->pub.publish(msg);
}

//////////////////////////////////////////////////////////////
