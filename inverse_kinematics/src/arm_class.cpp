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
Arm::Arm() : gripper() 
{
    this->init();
    this->arm_side = RIGHT;
}

// CONSTRUCTOR
// does the ros initialization and 
// calibrates/opens the gripper if necessary 
Arm::Arm(ros::NodeHandle handle, bool arm_side) : gripper(handle, arm_side)
{
    this->init();
    this->arm_side = arm_side;

    string pub_joint_topic;
    if (this->arm_side == LEFT) pub_joint_topic = "/robot/limb/left/joint_command";
    else pub_joint_topic = "/robot/limb/right/joint_command";

    string pub_point_topic;
    if (this->arm_side == LEFT) pub_point_topic = "/left/goal_point";
    else pub_point_topic = "/right/goal_point";

    string sub_topic;
    if (this->arm_side == LEFT) sub_topic = "/robot/limb/left/endpoint_state";
    else sub_topic = "/robot/limb/right/endpoint_state";
   
    this->order_pub = handle.advertise<baxter_core_msgs::JointCommand>(pub_joint_topic, 10);
    this->point_pub = handle.advertise<geometry_msgs::Pose>(pub_point_topic, 10);
    this->joint_sub = handle.subscribe<sensor_msgs::JointState>("/robot/joint_states", 10, &Arm::joint_callback, this);
    this->point_sub = handle.subscribe<baxter_core_msgs::EndpointState>(sub_topic, 10, &Arm::point_callback, this);
}

// MOVE TO FUNCTION (integer argument)
// calls one of the private functions preloaded 
// with hardcoded joint states; moves the arms there
void Arm::move_to(int hardcoded_state) 
{
    switch (hardcoded_state) 
    {
        case HOME:
            this->send_home();
            break;

        case CENTER:
            this->bring_center();
            break;
    }
}

// MOVE TO FUNCTION (joint command argument)
// readies the arms, then sets the arm's orders to the new location
void Arm::move_to(baxter_core_msgs::JointCommand new_order) 
{
    this->get_ready();
    this->orders = new_order;
}

// TURN WRIST TO FUNCTION
// brings wrist to a specified position
void Arm::turn_wrist_to(float new_position, bool is_increment) 
{
    float increment = 0.02;
    if (new_position < 0) increment*= -1;

    baxter_core_msgs::JointCommand new_orders = this->orders;
    for (size_t i = 0; i < new_orders.names.size(); i++)
    {
        string name = (arm_side == LEFT ? "left_w2" : "right_w2");
        if (new_orders.names[i].find(name) != string::npos) 
        {
            if (is_increment) new_orders.command[i]+= increment; 
            else new_orders.command[i] = new_position; 
        }
    }

    this->move_to(new_orders);
}

// ADJUST ENDPOINT FUNCTION
// sets the position in the given direction of the endpoint 
// to a specified value, or if increment is true, adjusts 
// the position of the arm slightly in the specified direction
void Arm::adjust_endpoint(int direction, float new_position, bool is_increment) 
{
    float increment = 0.01;
    if (new_position > 0) increment*= -1;

    geometry_msgs::Pose new_pose = this->endpoint;

    switch(direction) 
    {
        case X:
            if (is_increment) new_pose.position.x+= increment;
            else new_pose.position.x = new_position;
            break;

        case Y:
            if (is_increment) new_pose.position.y+= increment;
            else new_pose.position.y = new_position;
            break;

        case Z:
            if (is_increment) new_pose.position.z+= increment;
            else new_pose.position.z = new_position;
            break;
    }

    this->point_pub.publish(new_pose);
}

// LOWER ARM FUNCTION
// lowers baxter's arm a bit
void Arm::lower_arm() 
{
    float positions[4] = { 0.1, 0.0, -0.07, -0.13 };
    geometry_msgs::Pose new_pose = this->endpoint;
      
    if (fabs(new_pose.position.z - positions[0]) < fabs(new_pose.position.z - positions[1])
     && fabs(new_pose.position.z - positions[0]) < fabs(new_pose.position.z - positions[2])
     && fabs(new_pose.position.z - positions[0]) < fabs(new_pose.position.z - positions[3]))
        new_pose.position.z = positions[1];

    else if (fabs(new_pose.position.z - positions[1]) < fabs(new_pose.position.z - positions[0])
          && fabs(new_pose.position.z - positions[1]) < fabs(new_pose.position.z - positions[2])
          && fabs(new_pose.position.z - positions[1]) < fabs(new_pose.position.z - positions[3]))
        new_pose.position.z = positions[2];

    else new_pose.position.z = positions[3];
        
    this->point_pub.publish(new_pose);
}

// MAKE ENDPOINTS PERPENDICULAR
// Adjusts the endpoints of the "bring center" function, 
// ensuring the endpoints are perpendicular
void Arm::make_endpoints_perpendicular() 
{
    geometry_msgs::Point point;
    point.x = 0.60;  
    point.y = 0.05; 
    point.z = 0.65; 

    geometry_msgs::Quaternion quaternion;
    quaternion.x =  0.000;  
    quaternion.y =  0.707;  
    quaternion.z =  0.707;
    quaternion.w =  0.000; 
   
    if (this->arm_side == LEFT) 
    {
        point.y = 0.02;
        quaternion.z*= -1.0;
    }

    geometry_msgs::Pose new_pose;
    new_pose.position = point;
    new_pose.orientation = quaternion;

    this->point_pub.publish(new_pose);
}

//////////////////////////////////////////////////////////////

// Private Helper Functions & Callbacks

// JOINT CALLBACK FUNCTION
// updates the current positions of the joints
// if not in accordance with the orders, publish commands
void Arm::joint_callback(const sensor_msgs::JointStateConstPtr& msg) 
{
    this->joints_initialized = true;

    this->current_joint_positions.clear();
    for (size_t i = 0; i < msg->position.size(); i++)
    {
        string side = (arm_side == LEFT ? "left" : "right");
        if (msg->name[i].find(side) != string::npos) this->current_joint_positions.push_back(msg->position[i]);
    }

    if (this->ready_ && !this->is_positioned()) 
    {
        this->done_ = false;
        this->order_pub.publish(this->orders);
    }

    else if (this->ready_ && this->is_positioned()) 
    {
        this->done_ = true;
        this->ready_ = false;
    }
}

// POINT CALLBACK FUNCTION
// updates the current endpoint of the arm
void Arm::point_callback(const baxter_core_msgs::EndpointStateConstPtr& msg) 
{
    this->point_initialized = true;
    this->endpoint = msg->pose;
}

// INIT FUNCTION
// common code in constructors
// sets all bools to false
void Arm::init() 
{
    this->joints_initialized = false;
    this->point_initialized = false;
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
            ROS_INFO("\tmoving %s from [%f] to [%f]", 
                  orders.names[i].c_str(), 
                  this->current_joint_positions[i], 
                  this->orders.command[i]);
            */
            return false;
        }
    }

    //ROS_INFO("\trepositioned...");
    return true;
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
    }
    
    new_orders.command.resize(new_orders.names.size());
    new_orders.command[0] = 0.0; 
    new_orders.command[1] = 1.3;
    new_orders.command[2] = 0.0;
    new_orders.command[3] = -0.8;
    new_orders.command[4] = 0.0; 
    new_orders.command[5] = 0.95;
    new_orders.command[6] = 0.0; 

    this->move_to(new_orders);
}

// BRING CENTER FUNCTION
// moves baxter's arms to a hard coded position
// that brings the cube to a center position for solving
void Arm::bring_center() 
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
        new_orders.command[0] = -1.179; 
        new_orders.command[1] =  1.492;
        new_orders.command[2] =  0.280;
        new_orders.command[3] = -0.994;
        new_orders.command[4] = -1.024; 
        new_orders.command[5] =  1.493;
        new_orders.command[6] =  0.000;
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
        new_orders.command[0] =  1.223; 
        new_orders.command[1] =  1.554;
        new_orders.command[2] = -0.205;
        new_orders.command[3] = -0.994;
        new_orders.command[4] =  1.069; 
        new_orders.command[5] =  1.465;
        new_orders.command[6] =  0.000;
    }
    
    this->move_to(new_orders);

    /*
    // solving position
    new_orders.command[0] = 1.0; 
    new_orders.command[1] = 1.5;
    new_orders.command[2] = 0.0;
    new_orders.command[3] = -0.4;
    new_orders.command[4] = 0.6; 
    new_orders.command[5] = 1.25;
    new_orders.command[6] = 0.0;
    */
}

//////////////////////////////////////////////////////////////
