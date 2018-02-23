////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: baxter_inspector_helpers.hpp
//
// Purpose: Contains the helper functions associated with the 
// inspect cube function of the baxter class
//
////////////////////////////////////////////////////////////////

#include "baxter_class.hpp"

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

    else if (this->bring_arms_center() && this->arms_ready()) 
    {
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
        this->holding_arm->turn_wrist_to(CW2);
        this->first = false;
    }
                
    else if (this->arms_ready()) 
    {
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

    if(!this->action_complete) this->action_complete = this->change_hands();

    else if (this->arms_ready() && count != 0) 
    {
        ROS_INFO("TURNING WRIST...");
        this->holding_arm->turn_wrist_to(CW2);
        this->count = 0;
    }

    else if (this->arms_ready() && this->count == 0) 
    {
        this->reader.get_colors();
        this->move_on("BACK FACE READ...", READ_FRONT);
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
        this->holding_arm->turn_wrist_to(UP);
        this->first = false;
    }

    else if (this->arms_ready())
    {
        this->reader.get_colors();
        this->move_on("FRONT FACE READ...", TURN_DEMO);
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

    return (this->count == 3);
}

////////////////////////////////////////////////////////////////
