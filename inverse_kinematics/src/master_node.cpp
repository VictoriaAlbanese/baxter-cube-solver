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

enum State {INITIALIZE, OVER_CUBE, FIX_ORIENTATION, FIX_POSITION, TEARDOWN, DONE};

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Make baxter smile :)
    FaceDisplay baxter(nh);
    baxter.make_face(HAPPY);

    // make some variables for later
    Arm left_arm(nh, LEFT);
    Arm right_arm(nh, RIGHT);
    IKS ik_solver(nh, RIGHT);
    SquareDetector detector(nh);
    float offset, offset_x, offset_y;
    int count = 0;

    // main program content
    State state = INITIALIZE;
    while (ros::ok() && state != DONE) 
    {
        // if everything is initialized, start doing the dirty
        if (left_arm.initialized() && right_arm.initialized()) 
        {
            switch (state) 
            {
                case INITIALIZE:
         
    		        // if the arms are in position, move on
                    if (left_arm.done() && right_arm.done() && count != 0) 
                    {
                        ROS_INFO("ARMS INITIALIZED...");
                        state = OVER_CUBE;
                        count = 0;
                    }
     
                    // otherwise, move the arms out of the way
                    else if (count == 0) 
                    { 
                        ROS_INFO("INITIALIZING ARMS...");
                        left_arm.send_home();
                        right_arm.send_home();
                        count++;
                    }

    	            break;
    
                case OVER_CUBE:
    
                    // if we are hovering over the cube, move to the next stage
                    if (left_arm.done() && right_arm.done() && count != 0) 
                    {
    		            ROS_INFO("ARM OVER CUBE...");
                        baxter.make_face(HAPPY);
                        state = FIX_ORIENTATION;
                        count = 0;
                    }
             
                    // use the iks to move over the cube
                    else if (count == 0) 
                    {
                        ROS_INFO("LOOKING FOR CUBE...");
                        baxter.make_face(THINKING);
                        right_arm.move_to(ik_solver.get_orders());
                        ROS_INFO("MOVING ARM OVER CUBE...");
                        ros::Duration(1.0).sleep();
                        count++;
                    }

                    break;
     
                case FIX_ORIENTATION:
                   
                    // if no squares are detected, move out of the point cloud  
                    // attempt moving over the cube again
                    if (detector.get_num_squares() == 0 && count == 0) 
                    {
                        ROS_INFO("NO SQUARES DETECTED, TRYING AGAIN...");
                        baxter.make_face(SAD);
                        state = INITIALIZE;
                    } 
    
                    else 
                    {
                        // kill the cloud, no longer needed 
                        ik_solver.kill_cloud();
                        count++;

                        // turn the wrist until cube is oriented correctly
                        ROS_INFO("FIXING ORIENTATION...");
                        baxter.make_face(THINKING);
                        if (fabs(offset * 180 / M_PI) < 55 || fabs(offset * 180 / M_PI) > 65)
                        {
                            offset = detector.get_angular_offset();
                            //ROS_INFO("\toffset is [%f] rads or [%f] degs", offset, offset * 180 / M_PI);
                            right_arm.turn_wrist(offset);
                        }
      
                        // otherwise, move on to the next phase 
                        else 
                        { 
                            ROS_INFO("ORIENTATION FIXED...");
                            baxter.make_face(HAPPY);
                            state = FIX_POSITION;
                            count = 0;
                        }
                    }

                    break;

                case FIX_POSITION:
                   
                    // turn the wrist until cube is oriented correctly
                    ROS_INFO("FIXING POSITION...");
                    baxter.make_face(THINKING);
                    if (fabs(offset_x) < 5)
                    {
                        offset_x = detector.get_x_offset();
                        ROS_INFO("\tx offset is [%f]", offset_x);
                        right_arm.adjust_endpoint_x(offset_x);
                    }
      
                    // otherwise, move on to the next phase 
                    else 
                    { 
                        ROS_INFO("POSITION FIXED...");
                        baxter.make_face(HAPPY);
                        state = TEARDOWN;
                        count = 0;
                    }

                    break;
                
                case TEARDOWN:
                     
    		        // if the arms are in position, move on
                    if (left_arm.done() && right_arm.done() && count != 0) 
                    {
                        ROS_INFO("ARMS RESET...");
                        state = DONE;
                        count = 0;
                    }
     
                    // otherwise, move the arms out of the way
                    else if (count == 0) 
                    { 
                        ROS_INFO("RESETTING ARMS...");
                        left_arm.send_home();
                        right_arm.send_home();
                        count++;
                    }

    	            break;
            }

        }

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    // reset the arms
    ROS_INFO("DONE...");

    return 0;
}

////////////////////////////////////////////////////////////////

