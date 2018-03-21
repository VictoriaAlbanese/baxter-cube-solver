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
    switch(this->count) 
    {
        case 0:
        case 1:
            this->bring_arm_center(this->holding_arm);
            break;

        case 2:
        case 3:
            this->bring_arm_center(this->other_arm);
            break;
    
        case 4:
            this->count = 0;
            this->reader.get_colors();
            this->move_on("BOTTOM FACE READ...", READ_TOP); 
            break;
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
        this->holding_arm->turn_wrist_to(CW2);
        this->first = false;
    }
                
    else 
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
    if(!this->action_complete) this->action_complete = this->change_hands();
    else 
    {
        if (this->first) 
        {
            this->holding_arm->turn_wrist_to(CW2);
            this->first = false;
        }

        else  
        {
            this->reader.get_colors();
            this->move_on("BACK FACE READ...", READ_FRONT);
        }
    }
}

// READ FRONT FUNCTION
// puts the cube in a position such that 
// the front face can be read
void Baxter::read_front()
{
    if (this->first) 
    {
        this->holding_arm->turn_wrist_to(UP);
        this->first = false;
    }

    else 
    {
        this->reader.get_colors();
        this->move_on("FRONT FACE READ...", TURN_DEMO);
    }
}

// BRING ARMS CENTER FUNCTION
// moves the specified arms into the correct position 
// for the cube's faces to be turned
bool Baxter::bring_arm_center(Arm * arm) 
{
    if (this->first) 
    {    
        if (arm->side() == LEFT) ROS_INFO("BRINGING LEFT ARM CENTER...");
        else ROS_INFO("BRINGING RIGHT ARM CENTER...");
        this->first = false;
    }

    switch(this->count % 2) 
    {
        case 0:
            ROS_INFO("...step 1");
            arm->move_to(CENTER);
            this->count++;
            ROS_INFO("...end step 1");
            break;

        case 1:
            ROS_INFO("...step 2");
            arm->set_endpoint(P_CENTER);
            if (arm->move_to(ENDPOINT)) this->count++;
            ROS_INFO("...end step 2");
            break;
    }

    return (this->count == 2);
}

// BRING ARM UP FUNCTION
// moves the specified arm into the correct position 
// for the cube's faces to be read
bool Baxter::bring_arm_up(Arm * arm) 
{
    if (this->first) 
    {    
        if (arm->side() == LEFT) ROS_INFO("BRINGING LEFT ARM UP...");
        else ROS_INFO("BRINGING RIGHT ARM UP...");
        this->first = false;
    }

    switch(this->count) 
    {
        case 0:
            arm->move_to(READ_UP);
            this->count = 1;
            break;

        case 1:
            arm->set_endpoint(P_READ_UP);
            if (arm->move_to(ENDPOINT)) this->count = 2;
            break;
    }

    if (this->count == 2) 
    {
        this->count = 0;
        return true;
    }
    else return false;
}

////////////////////////////////////////////////////////////////
