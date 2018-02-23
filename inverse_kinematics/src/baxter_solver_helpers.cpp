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
    if (this->first) 
    {
        ROS_INFO("CHANGING HANDS...");
        this->first = false;
    }

    if (this->count == 0) 
    {
        this->other_arm->turn_wrist_to(CW);
        this->count = 1;
    }
  
    else if (this->arms_ready() && this->count == 1) 
    {
        float offset_y = this->other_arm->get_endpoint_y();

        if (this->other_arm->side() == LEFT && offset_y > L_FIRM_HOLD) 
        {
            this->other_arm->adjust_endpoint(Y, 1, true);
            this->other_arm->move_to(ENDPOINT);
        }

        else if (this->other_arm->side() == RIGHT && offset_y < R_FIRM_HOLD) 
        {
            this->other_arm->adjust_endpoint(Y, -1, true);
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

        if (this->other_arm->side() == LEFT && offset_y < L_AWAY) 
        {
            this->other_arm->adjust_endpoint(Y, -1, true);
            this->other_arm->move_to(ENDPOINT);
        }

        else if (this->other_arm->side() == RIGHT && offset_y > R_AWAY) 
        {
            this->other_arm->adjust_endpoint(Y, 1, true);
            this->other_arm->move_to(ENDPOINT);
        }
          
        else this->count = 3;
    }

    return (this->arms_ready() && count == 3);
}

// TURN LEFT/RIGHT FUNCTION
// lets baxter turn the right face of the cube in the specified 
// side, changing hands if necessary, in the specified direction
void Baxter::lr_turn(bool side, float direction) 
{
    if (this->other_arm->side() != side) this->action_complete = this->change_hands();

    if (this->count == 0 && this->action_complete)
    {
        ROS_INFO("TURNING OTHER WRIST UP...");
        this->other_arm->turn_wrist_to(UP);
        this->count = 1;
    }

    if (this->arms_ready() && this->count == 1) 
    {
        ROS_INFO("MOVING TO SOFT HOLD...");
        float offset_y = this->other_arm->get_endpoint_y();
    
        if (side == RIGHT && offset_y > L_SOFT_HOLD) 
        {
            this->other_arm->adjust_endpoint(Y, 1, true);
            this->other_arm->move_to(ENDPOINT);
        }

        else if (side == LEFT && offset_y < R_SOFT_HOLD) 
        {
            this->other_arm->adjust_endpoint(Y, -1, true);
            this->other_arm->move_to(ENDPOINT);
        }

        else 
        {
            ros::Duration(0.5).sleep();
            this->other_arm->gripper.grip();
            ros::Duration(0.5).sleep();
            this->count = 2; 
        }
    }
    
    if (this->arms_ready() && this->count == 2) 
    {
        ROS_INFO("TURNING CUBE...");
        this->other_arm->turn_wrist_to(direction);
        this->count = 3;
    }

    if (this->arms_ready() && this->count == 3) 
    {
        ros::Duration(0.5).sleep();
        this->other_arm->gripper.release();
        ros::Duration(0.5).sleep();
        this->count = 4;
    }
 
    if (this->arms_ready() && this->count == 4) 
    {
        ROS_INFO("MOVING OTHER ARM AWAY...");
        float offset_y = this->other_arm->get_endpoint_y();
      
        if (side == RIGHT && offset_y > R_AWAY) 
        {
            this->other_arm->adjust_endpoint(Y, 1, true);
            this->other_arm->move_to(ENDPOINT);
        }

        else if (side == LEFT && offset_y < L_AWAY) 
        {
            ROS_INFO("\tleft current[%f] < goal[%f]", offset_y, L_SOFT_HOLD);
            this->other_arm->adjust_endpoint(Y, -1, true);
            this->other_arm->move_to(ENDPOINT);
        }

        else this->count = 5;
    }
  
    if (this->arms_ready() && this->count == 5) 
    {
        ROS_INFO("TURNING OTHER WRIST UP...");
        this->other_arm->turn_wrist_to(UP);
        this->count = 6;
    }

    if (this->arms_ready() && this->count == 6) 
    {
        this->move_on("DONE FOR NOW...", DONE);
        this->count = 0;
    }
}

////////////////////////////////////////////////////////////////
