////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: square_detector_class.hpp
//
// Purpose: Declares a class which finds squares in a ros
// image topic and handles a bunch of stuff related to getting
// baxter's hand centered around the square (aka the cube)
//
////////////////////////////////////////////////////////////////

#ifndef SQUARE_DETECTOR_CLASS_HPP
#define SQUARE_DETECTOR_CLASS_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#define THRESH 50
#define X_DESIRED 360
#define Y_DESIRED 135
#define WINDOW_NAME "Image Window"

using namespace cv;
using namespace std;

class SquareDetector
{
    private:
        
        // members
        bool initialized;
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub;
        image_transport::Publisher pub;
        vector< vector<Point> > squares;

        // functions
        void callback(const sensor_msgs::ImageConstPtr& msg);
        void find_squares(Mat& image);
        void draw_squares(Mat& image);
        double angle(Point pt1, Point pt2, Point pt0);
        Point get_centroid(vector<Point> square);


    public:
        
        // members
        // n/a
        
        // functions
        SquareDetector();
        SquareDetector(ros::NodeHandle handle);
        ~SquareDetector();
        int get_num_squares() { return this->squares.size(); }
        float get_angular_offset();
        float get_x_offset();
        float get_y_offset();
};

#endif // SQUARE_DETECTOR_CLASS_HPP

////////////////////////////////////////////////////////////////
