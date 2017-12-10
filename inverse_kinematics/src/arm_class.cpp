//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: arm_class.cpp
//
// Description: Declares a class where positions of baxter's
// arm joints can be manipulated 
//
//////////////////////////////////////////////////////////////

#include "arm_class.hpp"

//////////////////////////////////////////////////////////////

// Public Member Functions

// DEFAULT CONSTRUCTOR
// does not initialize ros
// makes a right arm by default
Arm::Arm() 
{
    this->init();
    this->arm_side = RIGHT;

    Gripper gripper;
    this->gripper = gripper;
}

// CONSTRUCTOR
// does the ros initialization and 
// calibrates/opens the gripper if necessary 
Arm::Arm(ros::NodeHandle handle, bool arm_side)
{
    this->init();
    this->arm_side = arm_side;
 
    Gripper gripper(handle, this->arm_side);
    this->gripper = gripper; 
    if (!this->gripper.calibrated()) this->gripper.calibrate();
    this->gripper.release();

    string pub_topic;
    if (this->arm_side == LEFT) pub_topic = "/robot/limb/left/joint_command";
    else pub_topic = "/robot/limb/right/joint_command";
   
    this->pub = handle.advertise<baxter_core_msgs::JointCommand>(pub_topic, 10);
    this->sub = handle.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Arm::joint_callback, this);
}

// MOVE TO FUNCTION
// sets new orders
// this causes baxter's arms to move
void Arm::move_to(baxter_core_msgs::JointCommand new_order) 
{
    this->get_ready();
    this->orders = new_order;
}

// TURN WRIST FUNCTION
// turns baxter's wrist joint to change the orientation
// of the grippers by a tiny bit in a direction given by the offset 
void Arm::turn_wrist(float offset) 
{
    float increment = 0.02;
    if (offset < 0) increment*= -1;
                  
    baxter_core_msgs::JointCommand new_orders = this->orders;

    for (size_t i = 0; i < new_orders.names.size(); i++)
    {
        string name = (arm_side == LEFT ? "left_w2" : "right_w2");
        if (new_orders.names[i].find(name) != string::npos) new_orders.command[i]+= increment;
    }

    this->move_to(new_orders);
}

// SEND HOME
// moves baxter's arms to a hard coded position
// outside the view of the point cloud
void Arm::send_home() 
{
    baxter_core_msgs::JointCommand new_orders;
    new_orders.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

    if (this->arm_side == LEFT) 
    {
        new_orders.names.push_back("left_e0");
        new_orders.names.push_back("left_e1");
        new_orders.names.push_back("left_s0");
        new_orders.names.push_back("left_s1");
        new_orders.names.push_back("left_w0");
        new_orders.names.push_back("left_w1");
        new_orders.names.push_back("left_w2");

        new_orders.command.resize(new_orders.names.size());
        new_orders.command[0] = 0.0;
        new_orders.command[1] = 1.3;
        new_orders.command[2] = 0.0; 
        new_orders.command[3] = -0.8;
        new_orders.command[4] = 0.0;
        new_orders.command[5] = 0.95; 
        new_orders.command[6] = 0.0;
    }

    else
    {
        new_orders.names.push_back("right_e0");
        new_orders.names.push_back("right_e1");
        new_orders.names.push_back("right_s0");
        new_orders.names.push_back("right_s1");
        new_orders.names.push_back("right_w0");
        new_orders.names.push_back("right_w1");
        new_orders.names.push_back("right_w2");

        new_orders.command.resize(new_orders.names.size());
        new_orders.command[0] = 0.0; 
        new_orders.command[1] = 1.3;
        new_orders.command[2] = 0.0;
        new_orders.command[3] = -0.8;
        new_orders.command[4] = 0.0; 
        new_orders.command[5] = 0.95;
        new_orders.command[6] = 0.0; 
    }

    this->move_to(new_orders);
}

//////////////////////////////////////////////////////////////

// Private Helper Functions & Callbacks

// JOINT CALLBACK FUNCTION
// updates the current positions of the joints
// if not in accordance with the orders, publish commands
void Arm::joint_callback(const sensor_msgs::JointStateConstPtr& msg) 
{
    this->initialized_ = true;

    this->current_joint_positions.clear();
    for (size_t i = 0; i < msg->position.size(); i++)
    {
        string side = (arm_side == LEFT ? "left" : "right");
        if (msg->name[i].find(side) != string::npos) this->current_joint_positions.push_back(msg->position[i]);
    }

    if (this->ready_ && !this->is_positioned()) 
    {
        this->done_ = false;
        this->pub.publish(this->orders);
    }

    else if (this->ready_ && this->is_positioned()) 
    {
        this->done_ = true;
        this->ready_ = false;
    }
}

// INIT FUNCTION
// common code in constructors
// sets all bools to false
void Arm::init() 
{
    this->initialized_ = false;
    this->ready_ = false;
    this->done_ = false;
}

// GET READY FUNCTION
// flips the necessary state bools of the arm
// which will prepare the joints to make a movement
void Arm::get_ready() 
{
    this->ready_ = true;
    this->done_ = false;
}

// IS POSITIONED FUNCTION
// checks to see if the current positions of the joints
// is in accordance with the orders
bool Arm::is_positioned() 
{
    for(size_t i = 0; i < this->orders.command.size(); i++)
    {
        if (fabs(this->orders.command[i] - this->current_joint_positions[i]) > 0.01 )
        {
            /* 
            ROS_INFO("moving %s from [%f] to [%f]", 
                  orders.names[i].c_str(), 
                  this->current_joint_positions[i], 
                  this->orders.command[i]);
            */
            return false;
        }
    }

    ROS_INFO("Repositioned...");
    return true;
}

//////////////////////////////////////////////////////////////
