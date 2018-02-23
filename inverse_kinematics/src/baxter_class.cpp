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

// DEFAULT CONSTRUCTOR
// essentially does nothing, here as a formality
Baxter::Baxter() 
{
    this->state = INITIALIZE;
    this->first = true;
    this->action_complete = false;
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
    //this->state = INITIALIZE;
    this->state = INSPECT_CUBE;
    this->first = true;
    this->action_complete = false;
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
    }
}

// TURNING DEMO FUNCTION
// performs turns in every direction as a demonstration
void Baxter::turning_demo() 
{
    switch(this->state) 
    {
        case RIGHT_TURN_CW:
            /*this->turn_right(CW);
            break;
        
        case LEFT_TURN_CW:
            */this->turn_left(CW);
            break;
        
        case RIGHT_TURN_CCW:
            //this->turn_right(CCW);
            break;
        
        case LEFT_TURN_CCW:
            //this->turn_left(CCW);
            break;

        case TEARDOWN:
            //this->reset_arms();
            break;
    }
}

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
    this->action_complete = false;
}

////////////////////////////////////////////////////////////////
