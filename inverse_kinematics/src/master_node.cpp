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

enum State 
{
    INITIALIZE, 
    OVER_CUBE, 
    FIX_ORIENTATION, 
    FIX_POSITION, 
    LOWERING,
    TEARDOWN, 
    DONE
};

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
                        ROS_INFO("ARMS RESET...");
                        state = OVER_CUBE;
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

                        // only do this stuff the first time we try and fix the orientation
                        if (count == 1) 
                        {
                            ROS_INFO("FIXING ORIENTATION...");
                            ROS_INFO("\tgoal offset is between 58 and 62 degrees");
                            baxter.make_face(THINKING);
                            offset = detector.get_angular_offset();
                        }
                        
                        // turn the wrist until cube is oriented correctly
                        if (fabs(offset * 180 / M_PI) < 58 || fabs(offset * 180 / M_PI) > 62)
                        {
                            offset = detector.get_angular_offset();
                            ROS_INFO("\toffset is [%f] degrees", offset * 180 / M_PI);
                            right_arm.turn_wrist(offset);
                            count++;
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
                               
                    // adjust the endpoint until the cube is positioned correctly ...
                    if (count == 0) 
                    {
                        ROS_INFO("FIXING POSITION...");
                        ROS_INFO("\tgoal offset is within 10px in both directions");
                        baxter.make_face(THINKING);
                        offset_x = detector.get_x_offset();
                        offset_y = detector.get_y_offset();
                    }

                    // ... in the x direction
                    if (fabs(offset_x) > 10)
                    {
                        offset_x = detector.get_x_offset();
                        ROS_INFO("\tx offset is [%f]", offset_x);
                        right_arm.adjust_endpoint_x(offset_x);
                        right_arm.move_to(ik_solver.get_orders());
                        count++;
                    }

                    // ... in the y direction
                    else if (fabs(offset_y) > 10)
                    {
                        offset_y = detector.get_y_offset();
                        ROS_INFO("\ty offset is [%f]", offset_y);
                        right_arm.adjust_endpoint_y(offset_y);
                        right_arm.move_to(ik_solver.get_orders());
                        count++;
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

                /*case LOWERING:
    
                    // if we are done lowering our arm, move to the next stage
                    if (left_arm.done() && right_arm.done() && count != 0) 
                    {
                        ROS_INFO("ARM LOWERED...");
                        baxter.make_face(HAPPY);
                        state = TEARDOWN;
                        count = 0;
                    }
             
                    // use the iks to lower the arm a bit
                    else if (count == 0) 
                    {
                        ROS_INFO("LOWERING ARM...");
                        baxter.make_face(THINKING);
                        right_arm.adjust_endpoint_x(offset_x);
                        right_arm.move_to(ik_solver.get_orders());
                        ros::Duration(1.0).sleep();
                        count++;
                    }
                                           
                    break;*/
                
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

