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

            if (this->holding_arm->side() == LEFT) ROS_INFO("holding arm is left");
            else ROS_INFO("holding arm is right");
 
            if (this->other_arm->side() == LEFT) ROS_INFO("other arm is left");
            else ROS_INFO("other arm is right");
           
            this->other_arm->turn_wrist_to(CW);
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
            if (side == LEFT && offset_y < L_AWAY) this->increment(LEFT, OPPOSITE, true);
            else if (side == RIGHT && offset_y > R_AWAY) this->increment(RIGHT, OPPOSITE, true);
            else this->count = 4;
            break;
    
        case 4:
            temp = this->holding_arm;
            this->holding_arm = this->other_arm;
            this->other_arm = temp;
            done = true;
            this->count = 0;
     
            if (this->holding_arm->side() == LEFT) ROS_INFO("holding arm is now left");
            else ROS_INFO("holding arm is now right");

            if (this->other_arm->side() == LEFT) ROS_INFO("other arm is now left");
            else ROS_INFO("other arm is now right");
    
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
       
        if (this->holding_arm->side() == LEFT) ROS_INFO("holding arm is still left");
        else ROS_INFO("holding arm is still right");
    
        if (this->other_arm->side() == LEFT) ROS_INFO("other arm is still left");
        else ROS_INFO("other arm is still right");
 
        switch(this->count) 
        {
            case 0:
                ROS_INFO("\tturning wrist up");
                this->other_arm->turn_wrist_to(UP);
                this->count = 1;
                break;
    
            case 1:
                ROS_INFO("\tmoving to soft hold");
                if (side == RIGHT) ROS_INFO("right  [%f] < [%f]", offset_y, R_SOFT_HOLD);
                else if (side == LEFT) ROS_INFO("left [%f] > [%f]", offset_y, L_SOFT_HOLD);

                if (side == LEFT && offset_y > L_SOFT_HOLD) this->increment(RIGHT, SAME);
                else if (side == RIGHT && offset_y < R_SOFT_HOLD) this->increment(LEFT, SAME);
                else this->count = 2; 
                break;
       
           case 2:  
                ROS_INFO("\tgripping & turning cube");
                ros::Duration(0.5).sleep();
                this->other_arm->gripper.grip();
                this->other_arm->turn_wrist_to(direction);
                this->count = 3;
                break;
    
           case 3:
                ROS_INFO("\treleasing cube");
                ros::Duration(0.5).sleep();
                this->other_arm->gripper.release();
                this->count = 4;
                break;
                  
            case 4:
                ROS_INFO("\tmoving away");
                if (side == RIGHT && offset_y > R_AWAY) this->increment(RIGHT, SAME, true); 
                else if (side == LEFT && offset_y < L_AWAY) this->increment(LEFT, SAME, true);
                else this->count = 5;
                break;
                    
            case 5: 
                ROS_INFO("\tturning wrist up");
                this->other_arm->turn_wrist_to(UP);
                this->count = 6;
                break;
    
            case 6:
                this->move_on("TURN COMPLETE...", INCREMENT);
                this->count = 0;
                break;
        }
    }
}

// INCREMENT FUNCTION
// increments the y coordinate of the endpoint of the 
// other arm in the specified direction (left/right)
void Baxter::increment(bool direction, bool side, bool do_it) 
{
    Arm * arm = this->other_arm;
    if (side == OPPOSITE) arm = this->holding_arm;

    if (arm->side() == LEFT) ROS_INFO("about to move the left side");
    else if (arm->side() == RIGHT) ROS_INFO("about to move the right side");

    if (direction == LEFT) 
    {
        if (!do_it) arm->adjust_endpoint(Y, -1, true);
        else arm->adjust_endpoint(Y, L_AWAY);
        while (!arm->move_to(ENDPOINT)) ros::spinOnce(); 
    }

    if (direction == RIGHT) 
    {
        if (!do_it) arm->adjust_endpoint(Y, 1, true);
        else arm->adjust_endpoint(Y, R_AWAY);
        while (!arm->move_to(ENDPOINT)) ros::spinOnce();
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
        else if (direction == CW2 || direction == CCW2) ROS_INFO("TURNING LEFT FACE 180 DEGREES");
    }

    if (side == RIGHT) 
    {
        if (direction == CW) ROS_INFO("TURNING RIGHT FACE CLOCKWISE 90 DEGREES");
        else if (direction == CCW) ROS_INFO("TURNING RIGHT FACE COUNTERCLOCKWISE 90 DEGREES");
        else if (direction == CW2 || direction == CCW2) ROS_INFO("TURNING RIGHT FACE 180 DEGREES");
    }
}

////////////////////////////////////////////////////////////////
