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
    bool done = false;
    bool side = this->other_arm->side();
    float offset_y = this->other_arm->get_endpoint_y();
    Arm * temp = NULL;
   
    switch(this->count) 
    {
        case 0:
            ROS_INFO("CHANGING HANDS...");
            if (side == LEFT) this->other_arm->turn_wrist_to(CW);
            else this->other_arm->turn_wrist_to(UP);
            this->count = 1;
            break;
                
        case 1:
            if (side == LEFT && offset_y > L_FIRM_HOLD) this->increment(RIGHT, SAME);
            else if (side == RIGHT && offset_y < R_FIRM_HOLD) this->increment(LEFT, SAME);
            else this->count = 2;
            break;

        case 2: 
            this->other_arm->gripper.grip();
            this->holding_arm->gripper.release();
            this->count = 3;
            break;

        case 3: 
            side = this->holding_arm->side();
            offset_y = this->holding_arm->get_endpoint_y();
            if (side == LEFT) this->increment(LEFT, OPPOSITE, L_AWAY);
            else if (side == RIGHT) this->increment(RIGHT, OPPOSITE, R_AWAY);
            this->count = 4;
            break;
    
        case 4:
            temp = this->holding_arm;
            this->holding_arm = this->other_arm;
            this->other_arm = temp;
            done = true;
            this->count = 0;
            ROS_INFO("CHANGING HANDS COMPLETE..");
            break;
    }
   
    return done;
}

// TURN LEFT/RIGHT FUNCTION
// lets baxter turn the right face of the cube in the specified 
// side, changing hands if necessary, in the specified direction
void Baxter::lr_turn(bool side, float direction) 
{
    if (this->other_arm->side() == side) this->action_complete = true;
    else this->action_complete = this->change_hands();
   
    float offset_y = this->other_arm->get_endpoint_y();
    if (this->action_complete)
    {
        if (this->first) 
        {
            ROS_INFO("PERFORMING TURN...");   
            this->first = false;
        }
 
        switch(this->count) 
        {
            case 0:
                this->other_arm->turn_wrist_to(UP);
                this->count = 1;
                break;
     
            case 1:
                if (side == LEFT) this->increment(LEFT, OPPOSITE, R_FIRM_HOLD);
                else if (side == RIGHT) this->increment(RIGHT, OPPOSITE, L_FIRM_HOLD);
                this->count = 2; 
                break;

            case 2:
                if (side == LEFT && offset_y > L_SOFT_HOLD) this->increment(RIGHT, SAME);
                else if (side == RIGHT && offset_y < R_SOFT_HOLD) this->increment(LEFT, SAME);
                else this->count = 3; 
                break;
       
           case 3:  
                ros::Duration(0.5).sleep();
                this->other_arm->gripper.grip();
                this->other_arm->turn_wrist_to(direction);
                this->count = 4;
                break;
    
           case 4:
                ros::Duration(0.5).sleep();
                this->other_arm->gripper.release();
                this->count = 5;
                break;
                  
            case 5:
                if (side == RIGHT) this->increment(RIGHT, SAME, R_AWAY); 
                else if (side == LEFT) this->increment(LEFT, SAME, L_AWAY);
                this->count = 6;
                break;
                    
            case 6: 
                if (side == LEFT) this->other_arm->turn_wrist_to(CW);
                else this->other_arm->turn_wrist_to(UP);
                this->count = 7;
                break;
    
            case 7:
                this->move_on("TURN COMPLETE...", INCREMENT);
                this->count = 0;
                break;
        }
    }
}

// INCREMENT FUNCTION
// increments the y coordinate of the endpoint of the 
// other arm in the specified direction (left/right)
void Baxter::increment(bool direction, bool side, float to_here) 
{
    Arm * arm = this->other_arm;
    if (side == OPPOSITE) arm = this->holding_arm;

    if (direction == LEFT) 
    {
        if (to_here == -1) arm->adjust_endpoint(Y, -1, true);
        else arm->adjust_endpoint(Y, to_here);
        while (!arm->move_to(ENDPOINT)) ros::spinOnce(); 
    }

    if (direction == RIGHT) 
    {
        if (to_here == -1) arm->adjust_endpoint(Y, 1, true);
        else arm->adjust_endpoint(Y, to_here);
        while (!arm->move_to(ENDPOINT)) ros::spinOnce();
    }
}

////////////////////////////////////////////////////////////////
