//////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: face_display_class.hpp
//
// Description: Declares a class which abstracts away the
// face image manipulation for baxter's "face" screen
//
//////////////////////////////////////////////////////////////

#ifndef FACE_DISPLAY_CLASS_HPP
#define FACE_DISPLAY_CLASS_HPP

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#define HAPPY 0
#define SAD 1
#define THINKING 2

using namespace cv;

class FaceDisplay
{
    private: 
        ros::Publisher pub;

    public:
        FaceDisplay();
        FaceDisplay(ros::NodeHandle handle); 
        void make_face(int emotion);
};

#endif // FACE_DISPLAY_CLASS_HPP

//////////////////////////////////////////////////////////////
