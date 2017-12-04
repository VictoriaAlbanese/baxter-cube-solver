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
#include "square_detector_class.hpp"

enum State {OVER_CUBE, FIX_ORIENTATION, DONE};

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

    // Make a square detector, and offset variable for later
    SquareDetector detector(nh);
    float offset;

    // set loop rate, spin once
    ros::Rate loop_rate(10);
    ros::spinOnce();

    // main program content
    State state = OVER_CUBE;
    while (ros::ok() && state != DONE) 
    {
        switch (state) 
        {
            case OVER_CUBE:
 
                // use the iks to move over the cube
                ROS_INFO("MOVING ARM OVER CUBE...");
                right_arm.move_to(ik_solver.get_orders());
 
		        // if we are hovering over the cube, move to the next stage
                if (right_arm.done) 
                {
		            ROS_INFO("ARM OVER CUBE...");
                    state = FIX_ORIENTATION;
                }
                
	            break;

            case FIX_ORIENTATION:
                
                // turn the wrist until cube is oriented correctly
                ROS_INFO("FIXING ORIENTATION...");

                while (fabs(offset * 180 / M_PI) < 55 || fabs(offset * 180 / M_PI) > 65)
                {
                    //ROS_INFO("FIXING ORIENTATION...");
                    offset = detector.get_angular_offset();
                    ROS_INFO("\toffset is [%f] rads or [%f] degs", offset, offset * 180 / M_PI);
                    right_arm.turn_wrist(offset);
                }

                ROS_INFO("ORIENTATION FIXED...");
                ros::Duration(5.0).sleep();
                state = DONE;

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

