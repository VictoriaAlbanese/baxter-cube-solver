#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int thresh = 50;
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

    public:
        ImageConverter() : it_(nh_)
        {
            // Subscrive to input video feed and publish output video feed
            image_sub_ = it_.subscribe("/cameras/right_hand_camera/image", 1, &ImageConverter::imageCb, this);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);

            cv::namedWindow(OPENCV_WINDOW);
        }

        ~ImageConverter()
        {
            cv::destroyWindow(OPENCV_WINDOW);
        }

        void imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Draw all the squares you can find on the video stream
            vector<vector<Point> > squares;
            findSquares(cv_ptr->image, squares);
            drawSquares(cv_ptr->image, squares);
            //cv::circle(cv_ptr->image, cv::Point(0, 0), 10, CV_RGB(255,0,0));
            //cv::circle(cv_ptr->image, cv::Point(0, 0), 100, CV_RGB(255,0,0));
            //cv::circle(cv_ptr->image, cv::Point(0, 0), 1000, CV_RGB(255,0,0));

            // Update GUI Window
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            cv::waitKey(3);

            // Output modified video stream
            image_pub_.publish(cv_ptr->toImageMsg());
        }

        // helper function:
        // finds a cosine of angle between vectors
        // from pt0->pt1 and from pt0->pt2
        static double angle( Point pt1, Point pt2, Point pt0 )
        {
            double dx1 = pt1.x - pt0.x;
            double dy1 = pt1.y - pt0.y;
            double dx2 = pt2.x - pt0.x;
            double dy2 = pt2.y - pt0.y;
            return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
        }

        // returns sequence of squares detected on the image.
        // the sequence is stored in the specified memory storage
        static void findSquares( const Mat& image, vector<vector<Point> >& squares )
        {
            squares.clear();
        
            Mat pyr, timg, gray0(image.size(), CV_8U), gray;
        
            // down-scale and upscale the image to filter out the noise
            pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
            pyrUp(pyr, timg, image.size());
            vector<vector<Point> > contours;
        
            // find squares in every color plane of the image
            for( int c = 0; c < 3; c++ )
            {
                int ch[] = {c, 0};
                mixChannels(&timg, 1, &gray0, 1, ch, 1);
        
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 3);
                
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, Mat(), Point(-1,-1), 2);
                imshow("gray", gray);
    
                // find contours and store them all as a list
                findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    
                vector<Point> approx;
    
                // test each contour
                for( size_t i = 0; i < contours.size(); i++ )
                {
                    // approximate contour with accuracy proportional
                    // to the contour perimeter
                    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
    
                    // square contours should 
                    //  - have 4 vertices after approximation
                    //  - relatively large area (to filter out noisy contours)
                    //  - be convex
                    if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)) )
                    {
                        double maxCosine = 0;
    
                        for( int j = 2; j < 5; j++ )
                        {
                            // find the maximum cosine of the angle between joint edges
                            double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                            maxCosine = MAX(maxCosine, cosine);
                        }
    
                        // if cosines of all angles are small
                        // (all angles are ~90 degree) then write quandrange
                        // vertices to resultant sequence
                        if( maxCosine < 0.3 )
                            squares.push_back(approx);
                    }
                }
            }
        }

        // the function draws all the squares in the image
        static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
        {
            for( size_t i = 0; i < squares.size(); i++ )
            {
                const Point* p = &squares[i][0];
                int n = (int)squares[i].size();
                polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, 8);
            }
        
            imshow(OPENCV_WINDOW, image);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    
    return 0;
}
