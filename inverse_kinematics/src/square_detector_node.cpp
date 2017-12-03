////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: square_detector_node.cpp
//
// Purpose: Detects position and orientation of the square 
// (aka the cube) on the table in reference to where the cube
// should be on the table (centered beneath the gripper and 
// with little to no rotation so that it looks like a square, 
// not a diamond)
//
////////////////////////////////////////////////////////////////

#include "square_detector_class.hpp"
#include "arm_class.hpp"

enum State {FIX_ORI, FIX_POS, LOWER, DONE};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_detector_node");
    ros::NodeHandle nh;

    SquareDetector detector(nh);
    Arm right_arm(nh, RIGHT);
    right_arm.done = true;

    // Create a joint publisher to send the arm to the desired point
    ros::Publisher joint_pub = nh.advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 10);
    ros::Rate loop_rate(10);
    ros::spinOnce();    

    State state = FIX_ORI;
    ROS_INFO("FIXING ORIENTATION...");
    while (ros::ok()) 
    {
        // Get the angular offset
        float offset = detector.get_angular_offset();

        switch(state) 
        {
            case FIX_ORI:

                // If the cube is oriented correctly, move to the next state
                if (!right_arm.done && fabs(offset) * 180 / M_PI > 75 && fabs(offset) * 180 / M_PI < 85) 
                {
                    ROS_INFO("ORIENTATION FIXED...");
                    right_arm.done = true;
                    state = DONE;
                }
                
                // Move the arm until cube is "straight" w/ respect to the camera
                else
                {
                    if (offset > 0) right_arm.turn_wrist(0.02);
                    else right_arm.turn_wrist(-0.02);
                }

                break;
                
            case FIX_POS:
                break;

            case LOWER:
                break;

            case DONE:
                ROS_INFO("ALL DONE...");
                return 0;
        }

        // Spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}

////////////////////////////////////////////////////////////////
