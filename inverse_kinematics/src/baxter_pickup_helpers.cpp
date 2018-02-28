////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: baxter_pickup_helpers.cpp
//
// Purpose: Contains the helper functions associated with the 
// pickup function of the baxter class
//
////////////////////////////////////////////////////////////////

#include "baxter_class.hpp"

// INITIALIZE ARMS FUNCTION
// makes sure the arms are initialized first
// moves arms to an initial predetermined position
// calibrates and opens the grippers
void Baxter::initialize_arms() 
{
    if (this->holding_arm->initialized() && this->other_arm->initialized()) 
    {
        if (this->first) 
        { 
            ROS_INFO("INITIALIZING ARMS...");
            this->holding_arm->move_to(HOME);
            this->other_arm->move_to(HOME);
            if (!this->holding_arm->gripper.calibrated()) this->holding_arm->gripper.calibrate();
            if (!this->other_arm->gripper.calibrated()) this->other_arm->gripper.calibrate();
            this->holding_arm->gripper.release();
            this->other_arm->gripper.release();
            this->first = false;
        }
                    
        else 
        {
            this->move_on("ARMS INITIALIZED...", OVER_CUBE); 
            this->display.make_face(THINKING);
            ROS_INFO("LOOKING FOR CUBE...");
        }
    }
}

// FIND CUBE FUNCTION
// gets the position of the rubiks cube with the iks 
// and moves the arm over it, moving on when done
void Baxter::find_cube() 
{
    if(!this->action_complete) this->action_complete = this->holding_arm->move_to(ENDPOINT);
    else 
    {
        if (this->first) 
        {
            this->display.make_face(HAPPY);
            this->first = false;
        }
        
        else this->move_on("ARM OVER CUBE...", CHECK_SQUARE); 
    }
}

// CHECK SQUARES FUNCTION
// looks to see that there is a square on the table
// will check up to 40 times before giving up and starting over, 
// otherwise if a square is found it moves on normally
void Baxter::check_squares() 
{ 
    if (this->detector.get_num_squares() != 0) 
    {
        this->count = 0;
        this->holding_arm->iks.kill_cloud();
        this->move_on("SQUARES DETECTED...", FIX_ORIENTATION); 
    } 
   
    else if (this->count == 40)
    {
        this->count = 0;
        this->display.make_face(SAD);
        this->move_on("NO SQUARES DETECTED...", INITIALIZE); 
    }

    else this->count++;
}

// FIX ORIENTATION
// given that we can see a square on the table, look at the angle of 
// the diagonal of that square; if within a certain range, move on, 
// otherwise turn the wrist to position the arm above the cube correctly
void Baxter::fix_orientation() 
{ 
    if (this->detector.get_num_squares() != 0) 
    {
        if (this->first) 
        {
            ROS_INFO("FIXING ORIENTATION...");
            ROS_INFO("\tgoal offset is between 58 and 62 degrees");
            this->display.make_face(THINKING);
            this->first = false;
        }
 
        float offset = this->detector.get_angular_offset();
        ROS_INFO("\toffset is [%f] degrees", offset);
        ros::Duration(0.1).sleep();
        
        if (fabs(offset) < 58.0 || fabs(offset) > 62.0) this->holding_arm->turn_wrist_to(offset, true);
        else this->move_on("ORIENTATION FIXED...", FIX_POSITION); 
    }
}

// FIX POSITION
// given that we can see a square on the table, look at the position
// of the centroid of that square; if within 10px of directly under the arm,
// move on, otherwise move the arm to position it above the cube correctly
void Baxter::fix_position()
{
    if (this->detector.get_num_squares() !=0) 
    {
        if (this->first) 
        {
            ROS_INFO("FIXING POSITION...");
            ROS_INFO("\tgoal offset is within 10px in both directions");
            this->display.make_face(THINKING);
            this-> first = false;
        }

        // xy directions are swapped from baxter to image
        float offset_x = this->detector.get_y_offset();
        float offset_y = this->detector.get_x_offset();
        ROS_INFO("\toffset is (%f, %f)", offset_x, offset_y);

        if (fabs(offset_x) > 10) 
        {
            this->holding_arm->adjust_endpoint(X, offset_x, true);
            this->holding_arm->move_to(ENDPOINT);
            ros::Duration(0.4).sleep();
        }

        else if (fabs(offset_y) > 10) 
        {
            this->holding_arm->adjust_endpoint(Y, offset_y, true);
            this->holding_arm->move_to(ENDPOINT);
            ros::Duration(0.4).sleep();
        }

        else this->move_on("POSITION FIXED...", LOWERING); 
    }
}

// LOWER ARM FUNCTION
// lowers the arm down by a certain interval, 
// then either loops back to fix the orientation 
// or moves on to grab the cube, if low enough
void Baxter::lower_arm() 
{
    if (this->first)
    {
        ROS_INFO("LOWERING ARM...");
        float poses[4] = { 0.02, -0.06, -0.14 };      
        geometry_msgs::Pose new_pose = this->holding_arm->get_endpoint();
        new_pose.position.z = poses[this->count];
        this->holding_arm->set_endpoint(new_pose.position, new_pose.orientation);
        if (this->right_arm.move_to(ENDPOINT)) this->first = false; 
    }
        
    else  
    {
        if (this->holding_arm->ready_for_pickup()) 
        {
            this->move_on("READY FOR PICKUP...", PICKUP); 
            this->count = 0;
        }

        else 
        {
            this->move_on("ARM LOWERED...", FIX_ORIENTATION); 
            this->count++;
        }
    }
}

// GRAB CUBE FUNCTION
// closes the grippers to grab the cube, then moves on
void Baxter::grab_cube() 
{
    if (this->first) 
    { 
        ROS_INFO("GRABBING CUBE...");
        ros::Duration(2.0).sleep();
        this->holding_arm->gripper.grip();
        this->display.make_face(HAPPY);
        this->first = false;
    }
            
    else this->move_on("CUBE GRABBED...", INSPECT_CUBE); 
}

// RESET ARMS FUNCTION
// brings arms back to initial position
void Baxter::reset_arms() 
{
    if (this->first) 
    { 
        ROS_INFO("RESETTING ARMS...");
        this->holding_arm->move_to(HOME);
        this->other_arm->move_to(HOME);
        this->first = false;
    }
                
    else this->move_on("ARMS RESET...", DONE); 
}

////////////////////////////////////////////////////////////////
