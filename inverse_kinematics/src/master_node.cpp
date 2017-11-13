////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: master_node.cpp
//
// Purpose: The master control node of the baxter cube solver
//
////////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include "arm_class.hpp"

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;

    // create a left and right arm
    Arm left_arm(nh, LEFT);
    Arm right_arm(nh, RIGHT);

    // set loop rate, spin once
    ros::Rate loop_rate(10);
    ros::spinOnce();

    // send the arms to their initial position
    if (ros::ok()) 
    {
        ROS_INFO("Sending arms home...");
        left_arm.send_home(SETUP);
        right_arm.send_home(SETUP);
    }

    // main program content
    int counter = 0;
    while (ros::ok() && counter < 50) 
    {
        ROS_INFO("%d", counter++);

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

////////////////////////////////////////////////////////////////

