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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_detector_node");
    SquareDetector detector;
    ros::spin();
    
    return 0;
}

////////////////////////////////////////////////////////////////
