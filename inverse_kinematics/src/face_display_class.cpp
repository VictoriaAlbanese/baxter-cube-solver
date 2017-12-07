//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: face_display_class.hpp
//
// Description: Declares a class which abstracts away the
// face image manipulation for baxter's "face" screen
//
//////////////////////////////////////////////////////////////

#include "face_display_class.hpp"
#include "ros/package.h"

// DEFAULT CONSTRUCTOR
// does literally nothing
FaceDisplay::FaceDisplay() {}

// CONSTRUCTOR
// sets up the ros publisher
FaceDisplay::FaceDisplay(ros::NodeHandle handle) 
{
    this->pub = handle.advertise<sensor_msgs::Image>("/robot/xdisplay", 10);
}

// MAKE FACE
// publishes the face with the given emotion
void FaceDisplay::make_face(int emotion) 
{
    string path = ros::package::getPath("inverse_kinematics");
    if (emotion == HAPPY) path+= "/images/happy.png";
    if (emotion == SAD) path+= "/images/sad.png";
    if (emotion == THINKING) path+= "/images/thinking.png";

    Mat mat = imread(path, CV_LOAD_IMAGE_COLOR);
    sensor_msgs::ImagePtr face = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat).toImageMsg();
  
    // the waits give time for the face to actually appear 
    ros::Duration(1.0).sleep(); 
    this->pub.publish(face);
    ros::Duration(1.0).sleep(); 
}

//////////////////////////////////////////////////////////////
