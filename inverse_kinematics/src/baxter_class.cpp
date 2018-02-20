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
    
    this->holding_arm = &this->right_arm;
    this->other_arm = &this->left_arm;
}

// CONSTRUCTOR
// initializes all of the member classes properly
Baxter::Baxter(ros::NodeHandle handle)
   : display(handle)
   , left_arm(handle, LEFT) 
   , right_arm(handle, RIGHT)
   , detector(handle) 
   , reader(handle)
{
    this->state = INITIALIZE;
    this->first = true;
    this->count = 0;
 
    this->holding_arm = &this->right_arm;
    this->other_arm = &this->left_arm;
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
    }
}

// INSPECT CUBE FUNCTION
// lifts the cube up to baxter's "face" and looks at four 
// of the sides; also checks the side facing the hand its in
void Baxter::inspect_cube() 
{
    switch(this->state) 
    {
        case READ_BOTTOM:
            this->read_bottom();
            break;

        case READ_TOP: 
            this->read_top();
            break;

        case READ_BACK:
            this->read_back();
            break;

        case READ_FRONT:
            this->read_front();
            break;
        case TEARDOWN:
            this->reset_arms();
            break;
    }
}

////////////////////////////////////////////////////////////////

// Private Methods & Callbacks

// MOVE ON FUNCTION
// move on to the next stage, wipe each iks, 
// update all necessary state variables
void Baxter::move_on(string message, int new_state) 
{
    ROS_INFO("%s", message.c_str());
    this->holding_arm->iks.uninitialize();
    this->other_arm->iks.uninitialize();
    this->state = new_state;
    this->first = true;
}
    	   
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
                    
        else if (this->arms_ready()) 
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
    if (this->first && this->holding_arm->move_to(ENDPOINT)) 
    {
        this->display.make_face(HAPPY);
        this->first = false;
    }
        
    else if (!this->first && this->arms_ready()) this->move_on("ARM OVER CUBE...", CHECK_SQUARE); 
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
        float z_position = this->holding_arm->get_endpoint_z();
        ROS_INFO("\toffset is (%f, %f)", offset_x, offset_y);
        ros::Duration(0.4).sleep();

        if (fabs(offset_x) > 10) 
        {
            this->holding_arm->adjust_endpoint(X, offset_x, true);
            this->holding_arm->move_to(ENDPOINT);
        }

        else if (fabs(offset_y) > 10) 
        {
            this->holding_arm->adjust_endpoint(Y, offset_y, true);
            this->holding_arm->move_to(ENDPOINT);
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
        this->holding_arm->adjust_endpoint(Z, poses[this->count]);
        
        if (this->right_arm.move_to(ENDPOINT))                
        {
            ROS_INFO("LOWERING ARM... %f", poses[this->count]);     
            this->first = false; 
        }
    }
        
    else if (this->arms_ready()) 
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

        ros::Duration(1.0).sleep();
        this->holding_arm->gripper.grip();
        this->display.make_face(HAPPY);

        this->first = false;
    }
            
    else if (this->arms_ready()) 
    {
        this->move_on("CUBE GRABBED...", INSPECT_CUBE); 
    }
}

// READ BOTTOM FUNCTION
// moves the arms into the right position for 
// reading in the bottom face of the cube's colors
// then reads in the colors on that face
void Baxter::read_bottom() 
{
    if (this->first) 
    {
        ROS_INFO("BRINGING ARMS CENTER...");
        this->first = false;
    }

    else if (this->arms_ready() && this->bring_arms_center()) 
    {
        ROS_INFO("READING BOTTOM FACE...");
        this->count = 0;
        this->reader.get_colors();
        this->move_on("BOTTOM FACE READ...", READ_TOP); 
    }
}

// READ TOP FUNCTION
// moves the arms into the right position for 
// reading in the top face of the cube's colors
// then reads in the colors on that face
void Baxter::read_top()
{
    if (this->first) 
    { 
        ROS_INFO("TURNING WRIST...");
        this->holding_arm->turn_wrist_to(3.0);
        this->first = false;
    }
                
    else if (this->arms_ready()) 
    {
        ROS_INFO("READING TOP FACE...");
        this->reader.get_colors();
        this->move_on("TOP FACE READ...", READ_BACK); 
    }
}

// READ BACK FUNCTION
// cube swaps hands, moves into the right position 
// for reading in the back face of the cube's colors
void Baxter::read_back() 
{
    if (this->first) 
    {
        ROS_INFO("CHANGING HANDS...");
        this->first = false;
    }

    else if (this->arms_ready() && this->change_hands())
    {
        ROS_INFO("READING BACK FACE...");
        this->count = 0;
        this->reader.get_colors();
        this->move_on("DONE FOR NOW...", READ_FRONT);
    }
}

// READ FRONT FUNCTION
// puts the cube in a position such that 
// the front face can be read
void Baxter::read_front()
{
    if (this->first) 
    {
        ROS_INFO("TURNING WRIST...");
        this->holding_arm->turn_wrist_to(-2.91);
        this->first = false;
    }

    else if (this->arms_ready())
    {
        ROS_INFO("READING FRONT FACE...");
        this->reader.get_colors();
        this->move_on("DONE FOR NOW...", TEARDOWN);
    }
}

// BRING ARMS CENTER FUNCTION
// moves the arms into the correct position 
// for the cube's faces to be read
bool Baxter::bring_arms_center() 
{
    if (this->count == 0) 
    {
        this->holding_arm->move_to(CENTER);
        this->other_arm->move_to(CENTER);
        this->count = 1;
    }

    else if (this->arms_ready() && this->count == 1) 
    { 
        this->holding_arm->set_endpoint(P_CENTER);
        if (this->holding_arm->move_to(ENDPOINT)) this->count = 2;
    }

    else if (this->arms_ready() && this->count == 2) 
    { 
        this->other_arm->set_endpoint(P_CENTER);
        if (this->other_arm->move_to(ENDPOINT)) this->count = 3;
    }

    return (count == 3);
}

// CHANGE HANDS FUNCTION
// goes through the motions which enable 
// baxter to grab the cube in his other hand
bool Baxter::change_hands() 
{
    if (this->count == 0) 
    {
        this->holding_arm->turn_wrist_to(-1.7);
        this->count = 1;
    }
            
    else if (this->arms_ready() && this->count == 1) 
    {
        float offset_y = this->other_arm->get_endpoint_y();
        
        if (offset_y > L_FIRM_HOLD) 
        {
            this->other_arm->adjust_endpoint(Y, 1, true);
            this->other_arm->move_to(ENDPOINT);
        }
          
        else 
        {
            this->other_arm->gripper.grip();
            ros::Duration(1.0).sleep();
            this->holding_arm->gripper.release();

            Arm * temp = holding_arm;
            holding_arm = other_arm;
            other_arm = temp;

            this->count = 2;
        } 
    }

    else if (this->arms_ready() && this->count == 2) 
    {
        float offset_y = this->other_arm->get_endpoint_y();

        if (offset_y > R_AWAY) 
        {
            this->other_arm->adjust_endpoint(Y, 1, true);
            this->other_arm->move_to(ENDPOINT);
        }
          
        else 
        {
            this->count = 3;
        } 
    }

    return (count == 3);
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
                
    else if (this->arms_ready()) this->move_on("ARMS RESET...", DONE); 
}

////////////////////////////////////////////////////////////////
