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
    : gripper() 
    , iks()
{
    this->init();
    this->arm_side = RIGHT;
}

// CONSTRUCTOR
// does the ros initialization and 
// calibrates/opens the gripper if necessary 
Arm::Arm(ros::NodeHandle handle, bool arm_side) 
    : gripper(handle, arm_side)
    , iks(handle, arm_side)
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
   
    this->order_pub = handle.advertise<baxter_core_msgs::JointCommand>(pub_joint_topic, 1);
    this->point_pub = handle.advertise<geometry_msgs::Pose>(pub_point_topic, 1);
    this->joint_sub = handle.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1, &Arm::joint_callback, this);
    this->point_sub = handle.subscribe<baxter_core_msgs::EndpointState>(sub_topic, 1, &Arm::point_callback, this);
}

// MOVE TO FUNCTION (integer argument)
// calls one of the private functions preloaded 
// with hardcoded joint states; moves the arms there
bool Arm::move_to(int hardcoded_state) 
{
    bool success = true;

    switch (hardcoded_state) 
    {
        case ENDPOINT:
           if (!this->iks.create_orders()) success = false;
           else this->move_to(this->iks.get_orders());
           break;

        case HOME:
            this->send_home();
            break;

        case CENTER:
            this->bring_center();
            break;

        case READ_UP:
            this->bring_up();
            break;
    }

    return success;
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

    /* 
    if (this->arm_side == LEFT) ROS_INFO("PUBLISHING TO LEFT ARM...");
    else ROS_INFO("PUBLISHING TO RIGHT ARM...");

    ROS_INFO("\told: p(%f, %f, %f) q(%f, %f, %f, %f)",
            new_pose.position.x,
            new_pose.position.y,
            new_pose.position.z,
            new_pose.orientation.x,
            new_pose.orientation.y,
            new_pose.orientation.z,
            new_pose.orientation.w);
    */

    switch(direction) 
    {
        case X:
            if (is_increment) new_pose.position.x+= increment;
            else new_pose.position.x = new_position;
            if (this->use_track)
            {
                new_pose.position.y = this->track.y;
                new_pose.position.z = this->track.z;
            }
            break;

        case Y:
            if (is_increment) new_pose.position.y+= increment;
            else new_pose.position.y = new_position;
            if (this->use_track)
            {
                new_pose.position.x = this->track.x;
                new_pose.position.z = this->track.z;
            }
            break;

        case Z:
            if (is_increment) new_pose.position.z+= increment;
            else new_pose.position.z = new_position;
            if (use_track)
            {
                new_pose.position.x = this->track.x;
                new_pose.position.y = this->track.y;
            }
            break;
    }

    /* 
    ROS_INFO("\tnew: p(%f, %f, %f) q(%f, %f, %f, %f)",
            new_pose.position.x,
            new_pose.position.y,
            new_pose.position.z,
            new_pose.orientation.x,
            new_pose.orientation.y,
            new_pose.orientation.z,
            new_pose.orientation.w);
    */

    this->point_pub.publish(new_pose);
    if(this->use_track) this->track = new_pose.position;
}

// SET ENDPOINT FUNCTION (integer argument)
// calls one of the private functions preloaded
// with a hardcoded endpoint; moves the arms there
void Arm::set_endpoint(int hardcoded_state) 
{
    geometry_msgs::Pose new_pose;
    
    switch(hardcoded_state) 
    {
        case P_CENTER:
            new_pose = this->center_perpendicularly();
            break;

        case P_READ_UP:
            new_pose = this->bring_up_perpendicularly();
            break;
    }

    this->point_pub.publish(new_pose);
    if(this->use_track) this->track = new_pose.position;
}

// SET ENDPOINT FUNCTION (geometry_msgs arguments)
// sets the endpoint of the arms
void Arm::set_endpoint(geometry_msgs::Point point, geometry_msgs::Quaternion quaternion) 
{
    geometry_msgs::Pose new_pose;
    new_pose.position = point;
    new_pose.orientation = quaternion;

    this->point_pub.publish(new_pose);
    if(this->use_track) this->track = new_pose.position;
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
    this->ready_ = true;
    this->done_ = false;
    this->use_track = false;
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
            // ROS_INFO("\tmoving %s from [%f] to [%f]", 
            // this->orders.names[i].c_str(), 
            // this->current_joint_positions[i], 
            // this->orders.command[i]);
            return false;
        }
    }

    // ROS_INFO("\trepos  %s from [%f] to [%f]", 
    // this->orders.names[i].c_str(), 
    // this->current_joint_positions[i], 
    // this->orders.command[i]);

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
        new_orders.command[6] =  1.500;
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

// BRING UP FUNCTION
// moves baxter's arms to a hard coded position
// that brings the cube to right in front of the camera for color inspection
void Arm::bring_up() 
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
        new_orders.command[0] = -1.482; 
        new_orders.command[1] =  2.340;
        new_orders.command[2] = -0.436;
        new_orders.command[3] =  0.071;
        new_orders.command[4] = -1.503; 
        new_orders.command[5] =  1.600;
        new_orders.command[6] = -1.250;
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
        new_orders.command[0] =  1.482; 
        new_orders.command[1] =  2.340;
        new_orders.command[2] =  0.436;
        new_orders.command[3] =  0.071;
        new_orders.command[4] =  1.503; 
        new_orders.command[5] =  1.507;
        new_orders.command[6] = -2.017;
    }
    
    this->move_to(new_orders);
}

// CENTERS ARMS PERPENDICULARLY FUNCTION
// adjusts the endpoints of the results of the "bring 
// center" function, ensuring the endpoints are perpendicular
geometry_msgs::Pose Arm::center_perpendicularly()
{
    this->use_track = true;

    geometry_msgs::Point point; 
    geometry_msgs::Quaternion quaternion;  
   
    if (this->arm_side == LEFT) 
    {
        point = this->set_p(0.60, 0.02, 0.65);
        quaternion = this->set_q(-0.5, -0.5, 0.5, -0.5);;
    }

    else 
    {
        point = this->set_p(0.60, 0.05, 0.65);
        quaternion = this->set_q(0.000, 0.707, 0.707, 0.000); 
    }

    geometry_msgs::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;

    return pose;
}

// BRINGS UP ARMS PERPENDICULARLY FUNCTION
// adjusts the endpoints of the results of the "bring 
// up" function, ensuring the endpoints are perpendicular
geometry_msgs::Pose Arm::bring_up_perpendicularly()
{
    this->use_track = true;

    geometry_msgs::Point point = this->set_p(0.33, 0.00, 0.76);
    geometry_msgs::Quaternion quaternion = this->set_q(0.00, 0.0, 1.0, 0.0); 
    
    geometry_msgs::Pose pose;
    pose.position = point;
    pose.orientation = quaternion;

    return pose;
}

// SET P FUNCTION
// nice point constructor 
geometry_msgs::Point Arm::set_p(float new_x, float new_y, float new_z) 
{
    geometry_msgs::Point p;
    p.x = new_x;
    p.y = new_y;
    p.z = new_z;

    return p;
}

// SET Q FUNCTION
// nice quaternion constructor
geometry_msgs::Quaternion Arm::set_q(float new_x, float new_y, float new_z, float new_w) 
{
    geometry_msgs::Quaternion q;
    q.x = new_x;
    q.y = new_y;
    q.z = new_z;
    q.w = new_w;

    return q;
}

//////////////////////////////////////////////////////////////
