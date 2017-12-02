////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: pcl_node.cpp
//
// Purpose: Gets the highest point on the table (aka the cube) 
//
////////////////////////////////////////////////////////////////

#include "pcl_class.hpp"

int main(int argc, char ** argv) 
{
	// Initialize ros
	ros::init(argc, argv, "plc_node");
	ros::NodeHandle nh;

    // Make the cloud
    Cloud cloud(nh);

    // Spin
    ros::spin();
}

////////////////////////////////////////////////////////////////
