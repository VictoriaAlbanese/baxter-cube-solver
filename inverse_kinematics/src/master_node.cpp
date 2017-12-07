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
#include "face_display_class.hpp"

enum State {OVER_CUBE, FIX_ORIENTATION, DONE};

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;

    // Make baxter smile :)
    FaceDisplay baxter(nh);
    baxter.make_face(HAPPY);

    // create, initialize, and move left 
    // and right arms to a starting position
    ROS_INFO("INITIALIZING ARMS...");
    Arm left_arm(nh, LEFT);
    Arm right_arm(nh, RIGHT);
    ROS_INFO("ARMS INITIALIZED...");
    
    // create an imverse kinematic solver
    // and wait for a location to be published
    ROS_INFO("LOOKING FOR CUBE...");
    baxter.make_face(THINKING);
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
                baxter.make_face(THINKING);
                right_arm.move_to(ik_solver.get_orders());
 
		        // if we are hovering over the cube, move to the next stage
                if (right_arm.done) 
                {
		            ROS_INFO("ARM OVER CUBE...");
                    baxter.make_face(HAPPY);
                    state = FIX_ORIENTATION;
                    ros::Duration(1.0).sleep();
                }
                
	            break;

            case FIX_ORIENTATION:
               
                // if no squares are detected, move out of the point cloud 
                // attempt moving over the cube again
                if (detector.get_num_squares() == 0) 
                {
                    ROS_INFO("NO SQUARES DETECTED, TRYING AGAIN...");
                    baxter.make_face(SAD);
                    right_arm.send_home();
                    state = OVER_CUBE;
                } 

                else 
                {
                    // kill the cloud, no longer needed 
                    ik_solver.kill_cloud();
    
                    // turn the wrist until cube is oriented correctly
                    ROS_INFO("FIXING ORIENTATION...");
                    baxter.make_face(THINKING);
                    while (fabs(offset * 180 / M_PI) < 55 || fabs(offset * 180 / M_PI) > 65)
                    {
                        offset = detector.get_angular_offset();
                        //ROS_INFO("\toffset is [%f] rads or [%f] degs", offset, offset * 180 / M_PI);
                        right_arm.turn_wrist(offset);
                    }
    
                    ROS_INFO("ORIENTATION FIXED...");
                    baxter.make_face(HAPPY);
                    state = DONE;
                }

                break;
        }

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    // reset the arms
    left_arm.send_home();
    right_arm.send_home();
    ROS_INFO("DONE...");

    return 0;
}

////////////////////////////////////////////////////////////////

