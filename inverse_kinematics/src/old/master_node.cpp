////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: master_node.cpp
//
// Purpose: The master control node of the baxter cube solver
//
////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <string>

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

void move_on(string message, State new_state, bool * first, State * state, int * count); 
void initialize_arms(Arm * left, Arm * right);
void reset_arms(Arm * left, Arm * right);
void find_cube(Arm * right, IKS iks, FaceDisplay display); 
bool within_orientation_threshhold(float offset);
float fix_orientation(Arm * right, SquareDetector * detector);
float fix_x_position(float offset_y, Arm * right, IKS iks, SquareDetector * detector);
float fix_y_position(float offset_x, Arm * right, IKS iks, SquareDetector * detector);
float lower_arm(Arm * right, IKS iks, bool do_it); 
void grab_cube(Arm * right, FaceDisplay display); 

using std::string;

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Make baxter smile :)
    FaceDisplay display(nh);
    display.make_face(HAPPY);

    // big variables for later
    Arm left_arm(nh, LEFT);
    Arm right_arm(nh, RIGHT);
    IKS ik_solver(nh, RIGHT);
    SquareDetector detector(nh);
    float offset, offset_x, offset_y, offset_z = 0.1;
    
    // state variables
    State state = INITIALIZE;
    bool first = true;
    int count = 0;

    // main program content
    while (ros::ok() && state != DONE) 
    {
        switch (state) 
        {
            case INITIALIZE:
 
                // make sure the arms are initialized before you do anything
                if (left_arm.initialized() && right_arm.initialized()) 
                {
                    // first, move the arms out of the way
                    if (first) 
                    { 
                        ROS_INFO("INITIALIZING ARMS...");
                        initialize_arms(&left_arm, &right_arm);
                        first = false;
                    }
                    
                    // then, if the arms are in position, move on
                    else if (left_arm.done() && right_arm.done()) 
                    {
                        move_on("ARMS INITIALIZED...", OVER_CUBE, &first, &state, &count); 
                    }
                }
    	   
                break;
    
            case OVER_CUBE:
    
                // first, use the iks to find the cube
                if (first) 
                {
                    ROS_INFO("LOOKING FOR CUBE...");
                    find_cube(&right_arm, ik_solver, display);
                    first = false;
                }
    
                // then, if the arms are in position, move on 
                else if (left_arm.done() && right_arm.done()) 
                {
                    move_on("ARM OVER CUBE...", CHECK_SQUARE, &first, &state, &count); 
                }

                break;
     
            case CHECK_SQUARE:
               
                // if a square is found, kill the cloud and move on
                if (detector.get_num_squares() != 0) 
                {
                    ik_solver.kill_cloud();
                    move_on("SQUARES DETECTED...", FIX_ORIENTATION, &first, &state, &count); 
                } 
   
                // or if we have tried 40 times to find squares, move on
                else if (count == 40)
                {
                    display.make_face(SAD);
                    move_on("NO SQUARES DETECTED...", INITIALIZE, &first, &state, &count); 
                }

                // otherwise, spin 
                else count++;

                break;

            case FIX_ORIENTATION:
               
                // given that the squares appear
                if (detector.get_num_squares() != 0) 
                {
                    // first, print some nice happy info
                    if (first) 
                    {
                        ROS_INFO("FIXING ORIENTATION...");
                        ROS_INFO("\tgoal offset is between 58 and 62 degrees");
                        offset = detector.get_angular_offset();
                        ROS_INFO("\toffset is [%f] degrees", offset * 180 / M_PI);
                        display.make_face(THINKING);
                        first = false;
                    }
                    
                    // then, turn the wrist until oriented correctly and move on
                    if (!within_orientation_threshhold(offset)) offset = fix_orientation(&right_arm, &detector);
                    else move_on("ORIENTATION FIXED...", FIX_POSITION, &first, &state, &count); 
                }

                break;

            case FIX_POSITION:
                
                // given that the squares appear
                if (detector.get_num_squares() != 0) 
                {
                    // first, print some nice happy info
                    if (first) 
                    {
                        ROS_INFO("FIXING POSITION...");
                        ROS_INFO("\tgoal offset is within 10px in both directions");
                        offset_x = detector.get_x_offset();
                        offset_y = detector.get_y_offset();
                        ROS_INFO("\toffset is (%f, %f)", offset_x, offset_y);
                        display.make_face(THINKING);
                        first = false;
                    }

                    // then, adjust the arm until positioned correctly and move on
                    if (fabs(offset_x) > 10) offset_x = fix_x_position(offset_y, &right_arm, ik_solver, &detector);
                    else if (fabs(offset_y) > 10) offset_y = fix_y_position(offset_x, &right_arm, ik_solver, &detector);
                    else move_on("POSITION FIXED...", LOWERING, &first, &state, &count); 
                }

                break;

            case LOWERING:
   
                // first, use the iks to lower the arm a bit
                if (first) 
                {
                    ROS_INFO("LOWERING ARM...");
                    if (offset_z < -0.06) offset_z = lower_arm(&right_arm, ik_solver, true);
                    else offset_z = lower_arm(&right_arm, ik_solver, false);
                    first = false;
                }

                // then, if the arms are in position, move on 
                else if (left_arm.done() && right_arm.done()) 
                {
                    if (offset_z < -0.13) move_on("READY FOR PICKUP...", PICKUP, &first, &state, &count); 
                    else move_on("ARM LOWERED...", FIX_ORIENTATION, &first, &state, &count); 
                }
                
                break;
             
            case PICKUP:
     
                // first, grab the cube
                if (first) 
                { 
                    ROS_INFO("GRABBING CUBE...");
                    grab_cube(&right_arm, display);
                    first = false;
                }
            
    	        // then, if the arms are in position, move on 
                else if (left_arm.done() && right_arm.done()) 
                {
                    move_on("CUBE GRABBED...", TEARDOWN, &first, &state, &count); 
                }
    
                break;

            case TEARDOWN:

                // first, move the arms out of the way
                if (first) 
                { 
                    ROS_INFO("RESETTING ARMS...");
                    reset_arms(&left_arm, &right_arm);
                    first = false;
                }
                
    	        // then, if the arms are in position, move on
                else if (left_arm.done() && right_arm.done()) 
                {
                    move_on("ARMS RESET...", DONE, &first, &state, &count); 
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

// MOVE ON FUNCTION
// move on to the next stage, updating all necessary state variables, etc
void move_on(string message, State new_state, bool * first, State * state, int * count) 
{
    ROS_INFO("%s", message.c_str());
    *state = new_state;
    *first = true;
    *count = 0;
}

////////////////////////////////////////////////////////////////

// INITIALIZE ARMS FUNCTION
// resets the arms, and calibrates and opens the grippers
void initialize_arms(Arm * left, Arm * right) 
{
    reset_arms(left, right);
    if (!left->gripper.calibrated()) left->gripper.calibrate();
    if (!right->gripper.calibrated()) right->gripper.calibrate();
    left->gripper.release();
    right->gripper.release();
}

// RESET ARMS FUNCTION
// sends both arms to the preset "home" position
void reset_arms(Arm * left, Arm * right) 
{
    left->send_home();
    right->send_home();
}

// FIND CUBE FUNCTION
// gets the position of the rubiks cube and moves the arm over it
//void find_cube(Arm * right, IKS iks, FaceDisplay display) 
void find_cube(Arm * right, IKS iks, FaceDisplay display) 
{
    display.make_face(THINKING);
    right->move_to(iks.get_orders());
    display.make_face(HAPPY);
}

// WITHIN ORIENTATION THRESHHOLD FUNCTION
// check to see if the orientation offset is within the acceptable threshhold
bool within_orientation_threshhold(float offset) 
{
    return (fabs(offset * 180 / M_PI) > 58 && fabs(offset * 180 / M_PI) < 62);
}

// FIX ORIENTATION
// get and return the rotational 
// offset, then turn the arm a bit
float fix_orientation(Arm * right, SquareDetector * detector) 
{ 
    float offset = detector->get_angular_offset();
    ROS_INFO("\toffset is [%f] degrees", offset * 180 / M_PI);
    right->turn_wrist(offset);

    return offset;
}

// FIX X POSITION
// get and return the offset in the x direction, 
// then adjust the position of the arm
float fix_x_position(float offset_y, Arm * right, IKS iks, SquareDetector * detector)
{
    float offset_x = detector->get_x_offset();
    ROS_INFO("\toffset is (%f, %f)", offset_x, offset_y);
    right->adjust_endpoint_x(offset_x);
    right->move_to(iks.get_orders());

    return offset_x;
}
 
// FIX Y POSITION
// get and return the offset in the y direction, 
// then adjust the position of the arm
float fix_y_position(float offset_x, Arm * right, IKS iks, SquareDetector * detector)
{
    float offset_y = detector->get_y_offset();
    ROS_INFO("\toffset is (%f, %f)", offset_x, offset_y);
    right->adjust_endpoint_y(offset_y);
    right->move_to(iks.get_orders());

    return offset_y;
}

// LOWER ARM FUNCTION
// lowers the arm down by a certain amount
// updates the z_offset of the arm with return value
float lower_arm(Arm * right, IKS iks, bool do_it) 
{
    float offset = right->lower_arm(do_it);
    right->move_to(iks.get_orders());
    ROS_INFO("\tz offset is [%f]", offset);
    ros::Duration(0.5).sleep();

    return offset;
}

// GRAB CUBE FUNCTION
// closes the grippers to grab the cube
void grab_cube(Arm * right, FaceDisplay display) 
{
    right->gripper.grip();
    display.make_face(HAPPY);
    ros::Duration(3.0).sleep();
    right->gripper.release();
    ros::Duration(3.0).sleep();
}

////////////////////////////////////////////////////////////////
