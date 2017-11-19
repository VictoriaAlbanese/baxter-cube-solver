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

enum State {INITIALIZING, FIND_CUBE};

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;

    // create a left and right arm, and an iks
    Arm left_arm(nh, LEFT);
    Arm right_arm(nh, RIGHT);
    IKS ik_solver(nh);

    // set loop rate, spin once
    ros::Rate loop_rate(10);
    ros::spinOnce();

    // main program content
    State state = INITIALIZING;
    ROS_INFO("INITIALIZING COMPLETE...");
    while (ros::ok()) 
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
                    state = FIND_CUBE;
               	    ROS_INFO("INITIALIZING COMPLETE...");
                }

                break;

            case FIND_CUBE:
    		
		// find the cube, get the iks for it
		baxter_core_msgs::JointCommand over_cube = ik_solver.get_solved_state();
		right_arm.move_to(over_cube);

		// move to the next stage
		if (left_arm.is_done && right_arm.is_done)
		{
		    state = INITIALIZE;
		    ROS_INFO("CUBE FOUND...");
		}

	        break;
        }

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

////////////////////////////////////////////////////////////////

