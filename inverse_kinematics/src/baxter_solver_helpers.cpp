////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: baxter_class.hpp
//
// Purpose: Contains the helper functions associated with 
// solving ithe cube (making tuens, changing hands, etc)
//
////////////////////////////////////////////////////////////////

#include "baxter_class.hpp"

// CHANGE HANDS FUNCTION
// goes through the motions which enable 
// baxter to grab the cube in his other hand
bool Baxter::change_hands() 
{
    bool side = this->other_arm->side();
    float offset_y = this->other_arm->get_endpoint_y();
    Arm * temp = holding_arm;
    
    switch(this->count) 
    {
        case 0:
            ROS_INFO("CHANGING HANDS...");
            this->other_arm->turn_wrist_to(CW);
            this->count = 1;
            break;
                
        case 1:
            if (side == LEFT && offset_y > L_FIRM_HOLD) this->increment(RIGHT);
            else if (side == RIGHT && offset_y < R_FIRM_HOLD) this->increment(LEFT);
            else this->count = 2;
            break;

        case 2: 
            this->other_arm->gripper.grip();
            this->holding_arm->gripper.release();
            holding_arm = other_arm;
            other_arm = temp;
            this->count = 3;
            break;

        case 3: 
            if (side == LEFT && offset_y < L_AWAY) this->increment(LEFT, true);
            else if (side == RIGHT && offset_y > R_AWAY) this->increment(RIGHT, true);
            else this->count = 4;
            break;
    }
     
    return (count == 4);
}

// TURN LEFT/RIGHT FUNCTION
// lets baxter turn the right face of the cube in the specified 
// side, changing hands if necessary, in the specified direction
void Baxter::lr_turn(bool side, float direction) 
{
    if (this->other_arm->side() == side) this->action_complete = true;
    else this->action_complete = this->change_hands();
    
    if (this->action_complete) this->turning_report(side, direction);     
    float offset_y = this->other_arm->get_endpoint_y();
    
    switch(this->count) 
    {
        case 0:
            this->other_arm->turn_wrist_to(UP);
            this->count = 1;
            break;

        case 1:
            if (side == LEFT && offset_y > L_SOFT_HOLD) this->increment(RIGHT);
            else if (side == RIGHT && offset_y < R_SOFT_HOLD) this->increment(LEFT);
            else this->count = 2; 
            break;
   
       case 2:  
            ros::Duration(0.5).sleep();
            this->other_arm->gripper.grip();
            this->other_arm->turn_wrist_to(direction);
            this->count = 3;
            break;

       case 3:
            ros::Duration(0.5).sleep();
            this->other_arm->gripper.release();
            this->count = 4;
            break;
              
        case 4:
            if (side == RIGHT && offset_y > R_AWAY) this->increment(RIGHT, true); 
            else if (side == LEFT && offset_y < L_AWAY) this->increment(LEFT, true);
            else this->count = 5;
            break;
                
        case 5: 
            this->other_arm->turn_wrist_to(UP);
            this->count = 6;
            break;

        case 6:
            this->move_on("TURN COMPLETE...", INCREMENT);
            this->count = 0;
            break;
    }
}

// INCREMENT FUNCTION
// increments the y coordinate of the endpoint of the 
// other arm in the specified direction (left/right)
void Baxter::increment(bool direction, bool do_it) 
{
    if (direction == LEFT) 
    {
        if (!do_it) this->other_arm->adjust_endpoint(Y, -1, true);
        else this->other_arm->adjust_endpoint(Y, L_AWAY);
        this->other_arm->move_to(ENDPOINT);
    }

    if (direction == RIGHT) 
    {
        if (!do_it) this->other_arm->adjust_endpoint(Y, 1, true);
        else this->other_arm->adjust_endpoint(Y, R_AWAY);
        this->other_arm->move_to(ENDPOINT);
    }
}

// TURNING REPORT FUNCTION
// literally just prints info... printing logic took up a 
// lot of space so it was refactored meaninglessly yay neatness
void Baxter::turning_report(bool side, float direction)
{
    if (side == LEFT) 
    {
        if (direction == CW) ROS_INFO("TURNING LEFT FACE CLOCKWISE 90 DEGREES");
        else if (direction == CCW) ROS_INFO("TURNING LEFT FACE COUNTERCLOCKWISE 90 DEGREES");
        else if (direction == CW2 || CCW2) ROS_INFO("TURNING LEFT FACE 180 DEGREES");
    }

    if (side == RIGHT) 
    {
        if (direction == CW) ROS_INFO("TURNING RIGHT FACE CLOCKWISE 90 DEGREES");
        else if (direction == CCW) ROS_INFO("TURNING RIGHT FACE COUNTERCLOCKWISE 90 DEGREES");
        else if (direction == CW2 || CCW2) ROS_INFO("TURNING RIGHT FACE 180 DEGREES");
    }
}

////////////////////////////////////////////////////////////////
