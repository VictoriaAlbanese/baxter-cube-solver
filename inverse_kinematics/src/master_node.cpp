////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: master_node.cpp
//
// Purpose: The master control node of the baxter cube solver
//
////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "baxter_class.hpp"

int main(int argc, char **argv)
{
    // initialize the node and create a node handle
    ros::init(argc, argv, "master_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    // Make baxter smile :)
    Baxter baxter(nh);
    baxter.display.make_face(HAPPY);

    // main program content
    while (ros::ok() && baxter.get_state() != DONE) 
    {
        // pickup the cube
        baxter.pickup_cube();

        // spin & sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("DONE...");
    return 0;
}

////////////////////////////////////////////////////////////////
