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

enum State {INITIALIZING, FIND_CUBE};

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

    // main program content
    State state = INITIALIZING;
    while (ros::ok()) 
    {
        switch (state) 
        {
            case INITIALIZING:
                
                left_arm.send_home();
                right_arm.send_home();
                
                if (left_arm.is_done && right_arm.is_done) 
                {
                    state = FIND_CUBE;
                }

                break;

            case FIND_CUBE:
                ROS_INFO("next stage");
                break;
        }

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

////////////////////////////////////////////////////////////////

