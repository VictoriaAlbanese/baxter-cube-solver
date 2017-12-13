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
    CHECK_SQUARE,
    FIX_ORIENTATION, 
    FIX_POSITION, 
    LOWERING,
    PICKUP,
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
    float offset, offset_x, offset_y, offset_z = 0.1;
    int count = 0;
    bool first = true;

    // main program content
    State state = INITIALIZE;
    while (ros::ok() && state != DONE) 
    {
        switch (state) 
        {
            case INITIALIZE:
 
                // if everything is initialized, start doing the dirty
                if (left_arm.initialized() && right_arm.initialized()) 
                {
    		        // if the arms are in position, move on
                    if (left_arm.done() && right_arm.done() && !first && right_arm.gripper.ready()) 
                    {
                        ROS_INFO("ARMS RESET...");
                        state = OVER_CUBE;
                        first = true;
                    }
     
                    // otherwise, move the arms out of the way
                    else if (first) 
                    { 
                        ROS_INFO("RESETTING ARMS...");
                        left_arm.send_home();
                        right_arm.send_home();
                        if (!right_arm.gripper.calibrated()) right_arm.gripper.calibrate();
                        right_arm.gripper.release();
                        first = false;
                    }
                }
    	        
                break;
    
            case OVER_CUBE:
    
                // if we are hovering over the cube, move to the next stage
                if (left_arm.done() && right_arm.done() && !first) 
                {
    		        ROS_INFO("ARM OVER CUBE...");
                    state = CHECK_SQUARE;
                    first = true;
                }
             
                // use the iks to move over the cube
                else if (first) 
                {
                    ROS_INFO("LOOKING FOR CUBE...");
                    baxter.make_face(THINKING);
                    right_arm.move_to(ik_solver.get_orders());
                    baxter.make_face(HAPPY);
                    ROS_INFO("MOVING ARM OVER CUBE...");
                    ros::Duration(1.0).sleep();
                    first = false;
                }

                break;
     
            case CHECK_SQUARE:
                
                // if no squares are detected after 10 frames, move out of 
                // the point cloud and make another attempt at moving over the cube 
                if (detector.get_num_squares() == 0 && count == 20) 
                {
                    ROS_INFO("NO SQUARES DETECTED, TRYING AGAIN...");
                    baxter.make_face(SAD);
                    state = INITIALIZE;
                    count = 0;
                    first = true;
                } 
   
                // otherwise, try again
                else if (detector.get_num_squares() == 0 && count < 20) count++;
                
                // otherwise, kill the point cloud and move on
                else 
                {
                    ROS_INFO("SQUARES DETECTED...");
                    ik_solver.kill_cloud();
                    baxter.make_face(HAPPY);
                    state = FIX_ORIENTATION;
                }

                break;

            case FIX_ORIENTATION:
               
                // given that the squares appear
                if (detector.get_num_squares() != 0) 
                {
                    // only do this stuff the first time we try and fix the orientation
                    if (first) 
                    {
                        ROS_INFO("FIXING ORIENTATION...");
                        ROS_INFO("\tgoal offset is between 58 and 62 degrees");
                        baxter.make_face(THINKING);
                        offset = detector.get_angular_offset();
                        first = false;
                    }
                            
                    // turn the wrist until cube is oriented correctly
                    if (fabs(offset * 180 / M_PI) < 58 || fabs(offset * 180 / M_PI) > 62)
                    {
                        offset = detector.get_angular_offset();
                        ROS_INFO("\toffset is [%f] degrees", offset * 180 / M_PI);
                        right_arm.turn_wrist(offset);
                    }
          
                    // otherwise, move on to the next phase 
                    else 
                    { 
                        ROS_INFO("ORIENTATION FIXED...");
                        state = FIX_POSITION;
                        first = true;
                    }
                }

                break;

            case FIX_POSITION:
                
                // given that the squares appear
                if (detector.get_num_squares() != 0) 
                {
                    // adjust the endpoint until the cube is positioned correctly ...
                    offset_x = detector.get_x_offset();
                    offset_y = detector.get_y_offset();
                    if (first) 
                    {
                        ROS_INFO("FIXING POSITION...");
                        ROS_INFO("\tgoal offset is within 10px in both directions");
                        first = false;
                    }
                    ROS_INFO("\toffset is (%f, %f)", offset_x, offset_y);
    
                    // ... in the x direction
                    if (fabs(offset_x) > 10)
                    {
                        offset_x = detector.get_x_offset();
                        right_arm.adjust_endpoint_x(offset_x);
                        right_arm.move_to(ik_solver.get_orders());
                    }
    
                    // ... in the y direction
                    else if (fabs(offset_y) > 10)
                    {
                        offset_y = detector.get_y_offset();
                        right_arm.adjust_endpoint_y(offset_y);
                        right_arm.move_to(ik_solver.get_orders());
                    }
          
                    // otherwise, move on to the next phase 
                    else 
                    { 
                        ROS_INFO("POSITION FIXED...");
                        state = LOWERING;
                        first = true;
                    }
                }

                break;

            case LOWERING:
  
                // arm is low enough, wait and move to done phase
                if (offset_z < -0.06) 
                {
                    ROS_INFO("READY FOR PICKUP...");
                    baxter.make_face(HAPPY);
                    state = PICKUP;
                    first = true;
                }

                else 
                {
                    // if we are done lowering our arm, move to the next stage
                    if (left_arm.done() && right_arm.done() && !first) 
                    {
                        ROS_INFO("ARM LOWERED...");
                        state = FIX_ORIENTATION;
                        first = true;
                    }
                 
                    // use the iks to lower the arm a bit
                    else if (first) 
                    {
                        ROS_INFO("LOWERING ARM...");
                        offset_z = right_arm.lower_arm();
                        right_arm.move_to(ik_solver.get_orders());
                        ROS_INFO("\tz offset is [%f]", offset_z);
                        ros::Duration(1.0).sleep();
                        first = false;
                    }
                }

                break;
             
            case PICKUP:
                
    	        // if the arms are in position, move on
                if (left_arm.done() && right_arm.done() && !first) 
                {
                    right_arm.gripper.grip();
                    ROS_INFO("CUBE GRABBED...");
                    ros::Duration(10.0).sleep();
                    state = TEARDOWN;
                    first = true;
                }
     
                // otherwise, move the arms out of the way
                else if (first) 
                { 
                    ROS_INFO("GRABBING CUBE...");
                    offset_z = right_arm.lower_arm(true);
                    right_arm.move_to(ik_solver.get_orders());
                    first = false;
                }

                break;

            case TEARDOWN:
                    
    	        // if the arms are in position, move on
                if (left_arm.done() && right_arm.done() && !first) 
                {
                    ROS_INFO("ARMS RESET...");
                    state = DONE;
                    first = true;
                }
     
                // otherwise, move the arms out of the way
                else if (first) 
                { 
                    ROS_INFO("RESETTING ARMS...");
                    left_arm.send_home();
                    right_arm.send_home();
                    first = false;
                }

             break;
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

