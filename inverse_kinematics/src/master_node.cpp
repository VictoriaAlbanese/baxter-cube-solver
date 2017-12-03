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
#include "iks_class.hpp"

enum State {OVER_CUBE, DONE};

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;

    // create, initialize, and move left 
    // and right arms to a starting position
    ROS_INFO("INITIALIZING ARMS...");
    Arm left_arm(nh, LEFT);
    Arm right_arm(nh, RIGHT);
    ROS_INFO("ARMS INITIALIZED...");
    
    // create an imverse kinematic solver
    // and wait for a location to be published
    ROS_INFO("LOOKING FOR CUBE...");
    IKS ik_solver(nh, RIGHT);
    ROS_INFO("CUBE FOUND...");

    // set loop rate, spin once
    ros::Rate loop_rate(10);
    ros::spinOnce();

    // main program content
    State state = OVER_CUBE;
    ROS_INFO("MOVING ARM OVER CUBE...");
    while (ros::ok() && state != DONE) 
    {
        switch (state) 
        {
            case OVER_CUBE:
   
                // find the cube, get the iks for it
                baxter_core_msgs::JointCommand over_cube = ik_solver.get_orders();
                right_arm.move_to(over_cube);

		        // move to the next stage
                if (left_arm.done && right_arm.done) 
                {
		            ROS_INFO("ARM OVER CUBE...");
                    state = DONE;
                }
                
	            break;
        }

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("DONE...");
    return 0;
}

////////////////////////////////////////////////////////////////

