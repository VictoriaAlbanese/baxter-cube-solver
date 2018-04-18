////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: color_reader_class.hpp
//
// Purpose: Declares a class which reads the colors of each 
// side of the Rubik's cube into memory
//
////////////////////////////////////////////////////////////////

#ifndef COLOR_READER_CLASS_HPP
#define COLOR_READER_CLASS_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sstream>

#include "universal_constants.hpp"

#define SIZE 400
#define SIDE 5
#define CR_WINDOW_NAME "Color Reader"

using namespace cv;
using namespace std;

class ColorReader
{
    private:
        
        // members
        bool initialized;
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub;
        image_transport::Publisher pub;
        vector<vector<int> > colors;
        Mat cube_image;
        int counter;

        // functions
        void callback(const sensor_msgs::ImageConstPtr& msg);
        void inspect_face(Mat& image);
        void print_colors();
        int extract_color(Mat& image, int x_offset, int y_offset); 
        vector<Rect> extract_rectangles(Mat& image, int color);
        Mat threshold_image(Mat image, int color);

    public:
        
        // functions
        ColorReader();
        ColorReader(ros::NodeHandle handle);
        ~ColorReader();
        vector<vector<int> > get_colors();
};

#endif // COLOR_READER_CLASS_HPP

////////////////////////////////////////////////////////////////
