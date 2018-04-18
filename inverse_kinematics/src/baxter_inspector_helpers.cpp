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
        this->bring_arm_up(this->holding_arm);
        this->first = false;
    }
            
    else 
    {
        this->display.make_face(THINKING);
        this->reader.get_colors();
        this->move_on("BOTTOM FACE READ...", READ_TOP); 
        this->count = 0;
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
        this->holding_arm->turn_wrist_to(1.2);
        this->first = false;
    }
                
    else 
    {
        this->reader.get_colors();
        this->move_on("TOP FACE READ...", SWAP_HANDS); 
        this->display.make_face(HAPPY);
    }
}

// SWAP HANDS FUNCTION
// cube swaps hands, which involves a series of movements 
// before the swapping of hands even begins
void Baxter::swap_hands() 
{
    switch (this->count) 
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
            this->move_on("MOVING TO CHANGE HANDS...", READ_BACK);
            this->count = 0;
            break;
    }
}

// READ BACK FUNCTION
// cube swaps hands, moves into the right position 
// for reading in the back face of the cube's colors
void Baxter::read_back() 
{
    if (!this->action_complete) this->action_complete = this->change_hands();
    
    else 
    {
        if (this->first) 
        {
            this->bring_arm_up(this->holding_arm);
            this->first = false;
        }
            
        else  
        {
            this->display.make_face(THINKING);
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
        this->holding_arm->turn_wrist_to(1.8);
        this->first = false;
    }

    else 
    {
        switch(this->count) 
        {
            case 1: 
                this->reader.get_colors();
                this->count = 2;
                break;

            case 2:
                this->bring_arm_center(this->holding_arm);
                this->count = 3;
                break;

            case 3:
                this->move_on("FRONT FACE READ...", TURN_DEMO);
                this->display.make_face(HAPPY);
                this->count = 0;
                break;
        } 
    }
}

// BRING ARM CENTER FUNCTION
// moves the specified arm into the correct position 
// for the cube's faces to be turned
bool Baxter::bring_arm_center(Arm * arm) 
{
    switch(this->count % 2) 
    {
        case 0:
            arm->move_to(CENTER);
            this->count++;
            break;

        case 1:
            arm->set_endpoint(P_CENTER);
            if (arm->move_to(ENDPOINT)) this->count++;
            break;
    }

    return (this->count == 2);
}

// BRING ARM UP FUNCTION
// moves the specified arm into the correct position 
// for the cube's faces to be read
bool Baxter::bring_arm_up(Arm * arm) 
{
    switch(this->count % 2) 
    {
        case 0:
            arm->move_to(READ_UP);
            this->count++;
            break;

        case 1:
            arm->set_endpoint(P_READ_UP);
            if (arm->move_to(ENDPOINT)) this->count++;
            break;
    }

    return (this->count == 2); 
}

////////////////////////////////////////////////////////////////
