////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: baxter_class.hpp
//
// Purpose: Defines a class which represents baxter the robot
// as a whole and all his joints and sensors
//
////////////////////////////////////////////////////////////////

#include "baxter_class.hpp"

////////////////////////////////////////////////////////////////

// DEFAULT CONSTRUCTOR
// essentially does nothing, here as a formality
Baxter::Baxter() 
{
    this->state = INITIALIZE;
    this->first = true;
    this->count = 0;
}

// CONSTRUCTOR
// initializes all of the member classes properly
Baxter::Baxter(ros::NodeHandle handle)
   : display(handle)
   , left_arm(handle, LEFT) 
   , right_arm(handle, RIGHT)
   , ik_solver(handle, RIGHT)
   , detector(handle) 
{
    this->state = INITIALIZE;
    this->first = true;
    this->count = 0;
}

// PICKUP CUBE FUNCTION
// does all of the work to pick up the cube 
void Baxter::pickup_cube() 
{
    switch (state) 
    {
        case INITIALIZE:
            this->initialize_arms();
            break;
    
        case OVER_CUBE:
            this->find_cube();
            break;
     
        case CHECK_SQUARE:
            this->check_squares();
            break;

        case FIX_ORIENTATION:
            this->fix_orientation();
            break;

        case FIX_POSITION:
            this->fix_position();
            break;

        case LOWERING:
            this->lower_arm();
                
            break;
             
        case PICKUP:
            this->grab_cube();
            break;

        case TEARDOWN:
            this->reset_arms();        
            break;
    }
}

////////////////////////////////////////////////////////////////

// Private Methods & Callbacks

// MOVE ON FUNCTION
// move on to the next stage, updating all necessary state variables, etc
void Baxter::move_on(string message, int new_state) 
{
    ROS_INFO("%s", message.c_str());
    this->ik_solver.uninitialize();
    this->state = new_state;
    this->first = true;
    this->count = 0;
}
    	   
// INITIALIZE ARMS FUNCTION
// makes sure the arms are initialized first, then 
// moves to an initial position, calibrates and opens 
// the grippers, and moves on when done
void Baxter::initialize_arms() 
{
    if (this->left_arm.initialized() && this->right_arm.initialized()) 
    {
        if (this->first) 
        { 
            ROS_INFO("INITIALIZING ARMS...");
            
            this->left_arm.send_home();
            this->right_arm.send_home();
            if (!this->left_arm.gripper.calibrated()) this->left_arm.gripper.calibrate();
            if (!this->right_arm.gripper.calibrated()) this->right_arm.gripper.calibrate();
            this->left_arm.gripper.release();
            this->right_arm.gripper.release();

            this->first = false;
        }
                    
        else if (this->arms_ready()) 
        {
            move_on("ARMS INITIALIZED...", OVER_CUBE); 
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
    if (this->first && this->ik_solver.create_orders()) 
    {
        this->right_arm.move_to(this->ik_solver.get_orders());
        this->display.make_face(HAPPY);
                  
        this->first = false;
    }
        
    else if (!this->first && this->arms_ready()) move_on("ARM OVER CUBE...", CHECK_SQUARE); 
}

// CHECK SQUARES FUNCTION
// looks to see that there is a square on the table
// will check up to 40 times before giving up and starting over, 
// otherwise if a square is found it moves on normally
void Baxter::check_squares() 
{ 
    if (this->detector.get_num_squares() != 0) 
    {
        this->ik_solver.kill_cloud();
        this->move_on("SQUARES DETECTED...", FIX_ORIENTATION); 
    } 
   
    else if (this->count == 40)
    {
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
        float offset, abs_offset;

        if (this->first) 
        {
            ROS_INFO("FIXING ORIENTATION...");
            ROS_INFO("\tgoal offset is between 58 and 62 degrees");
            this->display.make_face(THINKING);
            first = false;
        }
 
        offset = this->detector.get_angular_offset();
        abs_offset = fabs(offset);
        ROS_INFO("\toffset is [%f] degrees", offset);
        ros::Duration(0.1).sleep();
        
        if (abs_offset < 58.0 || abs_offset > 62.0) this->right_arm.turn_wrist(offset);
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
        float offset_x, offset_y;
        
        if (this->first) 
        {
            ROS_INFO("FIXING POSITION...");
            ROS_INFO("\tgoal offset is within 10px in both directions");
            this->display.make_face(THINKING);
            first = false;
        }

        offset_x = this->detector.get_x_offset();
        offset_y = this->detector.get_y_offset();
        ROS_INFO("\toffset is (%f, %f)", offset_x, offset_y);
        ros::Duration(0.1).sleep();

        if (fabs(offset_x) > 10) 
        {
            this->right_arm.adjust_endpoint_x(offset_x);
            if (this->ik_solver.create_orders())
                this->right_arm.move_to(this->ik_solver.get_orders());
        }

        else if (fabs(offset_y) > 10) 
        {
            this->right_arm.adjust_endpoint_y(offset_y);
            if (this->ik_solver.create_orders())
                this->right_arm.move_to(this->ik_solver.get_orders());
        }

        else move_on("POSITION FIXED...", LOWERING); 
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
        this->right_arm.lower_arm();
        if (this->ik_solver.create_orders())
        {
            ROS_INFO("LOWERING ARM...");
            this->right_arm.move_to(this->ik_solver.get_orders());
            first = false;
        }
    }
        
    else if (this->arms_ready()) 
    {
        if (this->right_arm.ready_for_pickup()) move_on("READY FOR PICKUP...", PICKUP); 
        else move_on("ARM LOWERED...", FIX_ORIENTATION); 
    }
}

// GRAB CUBE FUNCTION
// closes the grippers to grab the cube, then moves on
void Baxter::grab_cube() 
{
    if (this->first) 
    { 
        ROS_INFO("GRABBING CUBE...");
 
        this->right_arm.gripper.grip();
        this->display.make_face(HAPPY);
                
        this->first = false;
    }
            
    else if (this->arms_ready()) 
    {
        move_on("CUBE GRABBED...", TEARDOWN); 
    }
}

// RESET ARMS FUNCTION
// sends both arms to the preset "home" position
// and then moves on when done
void Baxter::reset_arms() 
{
    if (this->first) 
    { 
        ROS_INFO("RESETTING ARMS...");
    
        this->left_arm.send_home();
        this->right_arm.send_home();
    
        first = false;
    }
                
    else if (this->arms_ready()) move_on("ARMS RESET...", DONE); 
}

////////////////////////////////////////////////////////////////
