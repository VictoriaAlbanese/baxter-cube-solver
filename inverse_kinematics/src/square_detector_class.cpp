////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: square_detector_class.cpp
//
// Purpose: Implements a class which finds squares in a ros
// image topic and handles a bunch of stuff related to getting
// baxter's hand centered around the square (aka the cube)
//
////////////////////////////////////////////////////////////////

#include "square_detector_class.hpp"

// CONSTRUCTOR
// Subscribes to input video feed and publish output video feed
SquareDetector::SquareDetector() : it(nh)
{
    this->initialized = false;

    namedWindow(WINDOW_NAME);
    this->pub = it.advertise("/image_converter/output_video", 1);
    this->sub = it.subscribe("/cameras/right_hand_camera/image", 1, &SquareDetector::callback, this);

    while (!this->initialized) ros::spinOnce();
}

// CONSTRUCTOR
// Subscribes to input video feed and publish output video feed
SquareDetector::SquareDetector(ros::NodeHandle handle) : it(handle)
{
    this->initialized = false;

    namedWindow(WINDOW_NAME);
    this->pub = it.advertise("/image_converter/output_video", 1);
    this->sub = it.subscribe("/cameras/right_hand_camera/image", 1, &SquareDetector::callback, this);
    
    while (!this->initialized) ros::spinOnce();
}

// DESTRUCTOR
// Destroys the window
SquareDetector::~SquareDetector()
{
    destroyWindow(WINDOW_NAME);
}

// CALLBACK FUNCTION
// Gets the ros image and turns it into an opencv image 
// Does a bunch of work detecting and drawing squares etc.
void SquareDetector::callback(const sensor_msgs::ImageConstPtr& msg)
{
    this->initialized = true;

    // Try and get the image from the ros topic
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    // If it doesn't work, throw an exception
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Draw all the squares you can find on the video stream
    this->find_squares(cv_ptr->image);
    if (squares.size() != 0) this->draw_squares(cv_ptr->image);

    // Draw a circle at the desired centroid of the square should be
    circle(cv_ptr->image, Point(X_DESIRED, Y_DESIRED), 5, CV_RGB(255,0,0));

    // Update GUI Window
    imshow(WINDOW_NAME, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    pub.publish(cv_ptr->toImageMsg());
}

// ANGLE FUNCTION
// Finds the cosine of angle between vectors from pt0->pt1 and from pt0->pt2
double SquareDetector::angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

// FIND SQUARES FUNCTION
// gets sequence of squares detected on the image
void SquareDetector::find_squares(Mat& image)
{
    // variable declarations
    vector< vector<Point> > contours;
    vector<Point> approx;
    
    // make some images for later   
    Mat pyr_down;                   // this image will be downscaled
    Mat pyr_up;                     // this image will be upscaled
    Mat gray(image.size(), CV_8U);  // this image will be greyscale
    Mat dialated;                   // this image will be dialated
        
    // downscale and upscale the image to filter out noise
    pyrDown(image, pyr_down, Size(image.cols/2, image.rows/2));
    pyrUp(pyr_down, pyr_up, image.size());
        
    // find squares in every color plane of the image
    this->squares.clear();
    for(int c = 0; c < 3; c++)
    {
        // handle the color channel stuff
        int ch[] = {c, 0};
        mixChannels(&pyr_up, 1, &gray, 1, ch, 1);
        
        // canny & dialate to get nice clean, big contours
        Canny(gray, dialated, 0, THRESH, 3);
        dilate(dialated, dialated, Mat(), Point(-1,-1), 2);
        findContours(dialated, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    
        // test each contour
        for(size_t i = 0; i < contours.size(); i++)
        {
            // approximate contour with accuracy proportional to the contour perimeter
            approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);
    
            // square contours should 
            //  - have 4 vertices after approximation
            //  - relatively large area (to filter out noisy contours)
            //  - be convex
            if(approx.size() == 4 
                    && fabs(contourArea(Mat(approx))) > 1000 
                    && isContourConvex(Mat(approx)))
            {
                // find the maximum cosine of the angle between joint edges
                double max_cosine = 0;
                for(int j = 2; j < 5; j++)
                {
                    double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                    max_cosine = MAX(max_cosine, cosine);
                }
    
                // if max cosines of all angles are small 
                // then the angles are ~90 degrees; add to list
                if(max_cosine < 0.3)
                {
                    this->squares.push_back(approx);
                }
            }
        }
    }
}

// DRAW SQUARES FUNCTION
// the function draws all the squares in the image
void SquareDetector::draw_squares(Mat& image)
{
    // Draw the green lines around the square
    for( size_t i = 0; i < this->squares.size(); i++ )
    {
        const Point* p = &this->squares[i][0];
        int n = (int)this->squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, 8);
    }
    
    // Draw the corner points and centroid of square
    int x_centroid = 0, y_centroid = 0;
    for (size_t i = 0; i < this->squares.size(); i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            //ROS_INFO("Corner %d (%d, %d)", j, this->squares[i][j].x, this->squares[i][j].y);
            circle(image, this->squares[i][j], 3 * (j + 1), CV_RGB(0,0,255), -1);
        }    

        Point centroid = this->get_centroid(this->squares[i]);
        circle(image, centroid, 10, CV_RGB(0,255,0), -1);
    }
        
    imshow(WINDOW_NAME, image);
}

// GET CENTROID FUNCTION
// gets the centroid of the passed square
Point SquareDetector::get_centroid(vector<Point> square) 
{
    Point centroid;
    
    for (int i = 0; i < 4; i++) 
    {
        centroid.x+= square[i].x;
        centroid.y+= square[i].y;
    }    
    
    centroid.x/= 4;
    centroid.y/= 4;

    return centroid;
}

// GET ANGULAR OFFSET FUNCTION
// gets the angle that the cube is rotated
float SquareDetector::get_angular_offset() 
{
    while (this->squares.size() == 0) ros::spinOnce();

    float dx = this->squares[0][2].x - this->squares[0][0].x;
    float dy = this->squares[0][2].y - this->squares[0][0].y;
    
    return asin(dx / dy);
}
/*
// GET X OFFSET FUNCTION
// gets the offset in x of the cube's current position from its desired position
float SquareDetector::get_x_offset(Point centroid) 
{
    return centroid.x - X_DESIRED;
}

// GET Y OFFSET FUNCTION
// gets the offset in y of the cube's current position from its desired position
float SquareDetector::get_y_offset() 
{
    return centroid.y - Y_DESIRED;
}
*/
////////////////////////////////////////////////////////////////
