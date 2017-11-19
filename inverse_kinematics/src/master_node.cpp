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

enum State {INITIALIZING, FIND_CUBE, DONE};

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;

    // create a left and right arm, and an iks
    Arm left_arm(nh, LEFT);
    Arm right_arm(nh, RIGHT);
    IKS ik_solver(nh, RIGHT);

    // set loop rate, spin once
    ros::Rate loop_rate(10);
    ros::spinOnce();

    // main program content
    State state = INITIALIZING;
    ROS_INFO("INITIALIZING COMPLETE...");
    while (ros::ok() && state != DONE) 
    {
        switch (state) 
        {
            case INITIALIZING:
                
		        // send the arms to their starting positions
		        left_arm.send_home();
                right_arm.send_home();
                
		        // move to the next stage
                if (left_arm.is_done && right_arm.is_done) 
                {
           	        ROS_INFO("INITIALIZING COMPLETE...");
                    state = FIND_CUBE;
                }

                break;

            case FIND_CUBE:
 
                // falsify the arms
                left_arm.is_done = false;
                right_arm.is_done = false;
   		   
                // find the cube, get the iks for it
                baxter_core_msgs::JointCommand over_cube = ik_solver.get_orders();
                right_arm.move_to(over_cube);

		        // move to the next stage
                if (left_arm.is_done && right_arm.is_done) 
                {
		            ROS_INFO("CUBE FOUND...");
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

