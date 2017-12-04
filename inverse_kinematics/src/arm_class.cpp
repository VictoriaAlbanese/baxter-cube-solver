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
// does not initialize ros or the object
// makes a right arm by default because why not
Arm::Arm() 
{
    this->initialized = false;
    this->ready = false;
    this->done = false;
    
    this->arm_side = RIGHT;
}

// CONSTRUCTOR
// initializes ros, and the object by 
// waiting for the joint callback to be called
Arm::Arm(ros::NodeHandle handle, bool arm_side)
{
    this->initialized = false;
    this->ready = false;
    this->done = false;

    string pub_topic;
    if (arm_side == LEFT) pub_topic = "/robot/limb/left/joint_command";
    else pub_topic = "/robot/limb/right/joint_command";
    
    this->arm_side = arm_side; 
    this->pub = handle.advertise<baxter_core_msgs::JointCommand>(pub_topic, 10);
    this->sub = handle.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Arm::joint_callback, this);
    
    while (!this->initialized) ros::spinOnce();
    this->send_home();
}

// MOVE TO FUNCTION
// sets new orders
// this causes baxter's arms to move
void Arm::move_to(baxter_core_msgs::JointCommand new_order) 
{
    this->get_ready();
    this->execute_orders(new_order);
}

// TURN WRIST FUNCTION
// turns baxter's wrist joint to change the orientation
// of the grippers by a tiny bit in a direction given by the offset 
void Arm::turn_wrist(float offset) 
{
    this->get_ready();
    
    float increment = 0.01;
    if (offset < 0) increment*= -1;
                  
    baxter_core_msgs::JointCommand msg = this->orders;

    for (size_t i = 0; i < msg.names.size(); i++)
    {
        if (msg.names[i].find((arm_side == LEFT ? "left_w2" : "right_w2")) != std::string::npos)
        {   
            msg.command[i]+= increment;
        }   
    }

    this->execute_orders(msg);
}

//////////////////////////////////////////////////////////////

// Private Helper Functions & Callbacks

// JOINT CALLBACK FUNCTION
// updates the current positions of the joints
// if not in accordance with the orders, publish commands
void Arm::joint_callback(const sensor_msgs::JointStateConstPtr& msg) 
{
    this->initialized = true;

    this->current_joint_positions.clear();
    for (size_t i = 0; i < msg->position.size(); i++)
    {
        if (msg->name[i].find((arm_side == LEFT ? "left" : "right")) != std::string::npos)
        {   
            this->current_joint_positions.push_back(msg->position[i]);
        }   
    }

    if (this->ready && !this->is_positioned()) 
    {
        this->done = false;
        this->pub.publish(this->orders);
    }

    else if (this->ready && this->is_positioned()) 
    {
        ROS_INFO("\t%s arm positioned", (arm_side == LEFT ? "left" : "right"));
        this->done = true;
        this->ready = false;
    }
}

// GET READY FUNCTION
// flips the necessary state bools of the arm
// which will prepare the joints to make a movement
void Arm::get_ready() 
{
    this->ready = true;
    this->done = false;
}

// EXECUTE ORDERS FUNCTION
// sets new orders for the joints, and waits for 
// those orders to be completed before continuing
void Arm::execute_orders(baxter_core_msgs::JointCommand new_orders) 
{
    this->orders = new_orders;
    while (!this->done) ros::spinOnce();
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
            //ROS_INFO("moving %s from [%f] to [%f]", orders.names[i].c_str(), this->current_joint_positions[i], this->orders.command[i]);
            return false;
        }
    }

    //ROS_INFO("Repositioned...");
    return true;
}

// SEND HOME
// moves baxter's arms to a hard coded position
// outside the view of the point cloud
void Arm::send_home() 
{
   this->ready = true;
    
    baxter_core_msgs::JointCommand msg;
    msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

    if (this->arm_side == LEFT) 
    {
        msg.names.push_back("left_e0");
        msg.names.push_back("left_e1");
        msg.names.push_back("left_s0");
        msg.names.push_back("left_s1");
        msg.names.push_back("left_w0");
        msg.names.push_back("left_w1");
        msg.names.push_back("left_w2");

        msg.command.resize(msg.names.size());
        msg.command[0] = 0.0;
        msg.command[1] = 1.3;
        msg.command[2] = 0.0; 
        msg.command[3] = -0.8;
        msg.command[4] = 0.0;
        msg.command[5] = 0.95; 
        msg.command[6] = 0.0;
    }

    else
    {
        msg.names.push_back("right_e0");
        msg.names.push_back("right_e1");
        msg.names.push_back("right_s0");
        msg.names.push_back("right_s1");
        msg.names.push_back("right_w0");
        msg.names.push_back("right_w1");
        msg.names.push_back("right_w2");

        msg.command.resize(msg.names.size());
        msg.command[0] = 0.0; 
        msg.command[1] = 1.3;
        msg.command[2] = 0.0;
        msg.command[3] = -0.8;
        msg.command[4] = 0.0; 
        msg.command[5] = 0.95;
        msg.command[6] = 0.0; 
    }

    this->execute_orders(msg);
}

//////////////////////////////////////////////////////////////
