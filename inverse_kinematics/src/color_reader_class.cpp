////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: color_reader_class.cpp
//
// Purpose: Implements a class which reads the colors of each 
// side of the Rubik's cube into memory
//
////////////////////////////////////////////////////////////////

#include "color_reader_class.hpp"
string color_converter(int color); 

bool by_x_coord(Point p1, Point p2) { return p1.x < p2.x; };
bool by_y_coord(Point p1, Point p2) { return p1.y < p2.y; };

////////////////////////////////////////////////////////////////

// Public Members

// DEFAULT CONSTRUCTOR
// does ros initialization; subscribes to input 
// video feed and publishes output video feed
ColorReader::ColorReader() : it(nh)
{
    this->initialized = false;

    namedWindow(CR_WINDOW_NAME);
    this->pub = it.advertise("/color_reader/output_video", 1);
    this->sub = it.subscribe("/cameras/head_camera/image", 1, &ColorReader::callback, this);
    this->counter = 0;

    while (!this->initialized) ros::spinOnce();
}

// CONSTRUCTOR
// does ros initialization; subscribes to input 
// video feed and publishes output video feed
ColorReader::ColorReader(ros::NodeHandle handle) : it(handle)
{
    this->initialized = false;
    
    namedWindow(CR_WINDOW_NAME);
    this->pub = it.advertise("/color_reader/output_video", 1);
    this->sub = it.subscribe("/cameras/head_camera/image", 1, &ColorReader::callback, this);
    this->counter = 0;
    
    while (!this->initialized) ros::spinOnce();
}

// DESTRUCTOR
// destroys the window
ColorReader::~ColorReader()
{
    destroyWindow(CR_WINDOW_NAME);
}

// GET COLORS
// prints the colors in the current vector 
// and also returns a list of them
vector<vector<int> > ColorReader::get_colors() 
{ 
    for (int i = 0; i < 200; i++) ros::spinOnce();
    this->print_colors();
    ros::Duration(0.5).sleep();
    return this->colors; 
}

////////////////////////////////////////////////////////////////

// Private Members & Callbacks

// CALLBACK FUNCTION
// gets the ros image and turns it into an opencv image 
// detects and draws the squares, corners, centroids, etc
void ColorReader::callback(const sensor_msgs::ImageConstPtr& msg)
{
    this->initialized = true;
    
    // Try and get the image from the ros topic
    cv_bridge::CvImagePtr cv_ptr;
    try { cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Crop the image
    Rect rect(275, 115, 425, 325);
    Mat cropped = cv_ptr->image(rect);
    resize(cropped, cv_ptr->image, Size(SIZE, SIZE));

    // Get the color of each cubie
    this->inspect_face(cv_ptr->image);

    // Update window
    imshow(CR_WINDOW_NAME, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    pub.publish(cv_ptr->toImageMsg());
}

// INSPECT FACE FUNCTION
// fills out the colors of each cubie on the face of a rubik's cube
void ColorReader::inspect_face(Mat& image) 
{
    // some happy variables
    vector<Point> points;
    vector<pair<int, Rect> > rectangles;
    vector<Rect> temp;

    // make a list of pairs consisting of colors
    // and rectangles associated with that color
    for (int i = 1; i < 6; i++) 
    {
        temp = extract_rectangles(image, i);
        for (int j = 0; j < temp.size(); j++) 
        {
            if (temp[j].height != 0) rectangles.push_back(make_pair(i, temp[j])); 
        }
    }

    // Break up rectangles associated with more than one cubie
    for (int i = 0; i < rectangles.size(); i++) 
    {
        float x = rectangles[i].second.x;
        float y = rectangles[i].second.y;
        float h = rectangles[i].second.height;
        float w = rectangles[i].second.width;
        int h_num = round(h / 60.0);
        int w_num = round(w / 60.0);

        for (int j = 0; j < w_num; j++) 
        {
            for (int k = 0; k < h_num; k++) 
            {
                int p_x = x + w * ((j + 1) * 2 - 1) / (2 * w_num);
                int p_y = y + h * ((k + 1) * 2 - 1) / (2 * h_num);
                points.push_back(Point(p_x, p_y));
            }
        }
    }

    // Find the smallest x/y positions
    float smallest_x = INT_MAX;
    float smallest_y = INT_MAX;
    for (int i = 0; i < points.size(); i++) 
    {
        if (points[i].x < smallest_x) smallest_x = points[i].x;
        if (points[i].y < smallest_y) smallest_y = points[i].y;
    }

    // Subtract smallest x/y positions from all keypoints to "normalize"
    for (int i = 0; i < points.size(); i++) 
    {
        points[i].x-= smallest_x;
        points[i].y-= smallest_y;
    }
     
    // Fill out a vector with all white
    vector<vector<int> > face_colors;
    vector<int> white_vector;
    for (int i = 0; i < 3; i++) white_vector.push_back(WHITE);
    for (int i = 0; i < 3; i++) face_colors.push_back(white_vector);

    // Print info on each keypoint and fill the face
    for (int i = 0; i < points.size(); i++) 
    {
        float x = points[i].x;
        float y = points[i].y;
        float row = round(y / 60.0);
        float col = round(x / 60.0);

        /*
        ROS_INFO("================");
        ROS_INFO("\tx: %f", x); 
        ROS_INFO("\ty: %f", y); 
        ROS_INFO("\trow: %f col: %f", row, col); 
        ROS_INFO("\t%s", color_converter(extract_color(image, x + smallest_x, y + smallest_y)).c_str());
        */

        if (row < 3 && col < 3) face_colors[row][col] = extract_color(image, x + smallest_x, y + smallest_y);
    }

    this->colors = face_colors;
    //print_colors();
    //ros::Duration(2).sleep();
    imshow("centers", image);
}

// EXTRACT COLOR FUNCTION
// detects blobs of the specified color on the specified 
// image and returns infomration about the blobs as keypoints
vector<Rect> ColorReader::extract_rectangles(Mat& image, int color) 
{
    // Convert original to hsv, filter by color
    Mat hsv_image, filtered_image;
    cvtColor(image, hsv_image, COLOR_BGR2HSV); 
    filtered_image = threshold_image(hsv_image, color);

    // Morphological opening & closing to filter out noise and close holes
    erode(filtered_image, filtered_image, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
    dilate(filtered_image, filtered_image, getStructuringElement(MORPH_ELLIPSE, Size(10, 10))); 
    erode(filtered_image, filtered_image, getStructuringElement(MORPH_ELLIPSE, Size(15, 15)));

    // Find the edges of the color blob
    Mat edge, draw;
    Canny(filtered_image, edge, 50, 150, 3);
    dilate(edge, edge, getStructuringElement(MORPH_ELLIPSE, Size(20, 20))); 
    edge.convertTo(draw, CV_8U);

    // Extract contours of the canny image
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(draw, contours, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE);
    Mat output = image.clone();

    // Get the bounding rectangles about each blob
    vector<vector<Point> > contours_poly(contours.size());
    vector<Rect> bounding_rectangles(contours.size());
    vector<Point2f> centers(contours.size());
    vector<float> radii(contours.size());
    for (int i = 0; i < contours.size(); i++ )
    {
        if (contourArea(contours[i]) < 1000) continue;  // ignore contours that are too small
        if (hierarchy[i][3] < 0) continue;              // ignore "outer" contours

        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        bounding_rectangles[i] = boundingRect(Mat(contours_poly[i]));
    }

    // Draw each bounding rectangle
    for (int i = 0; i < bounding_rectangles.size(); i++ )
    {
        rectangle(output, bounding_rectangles[i].tl(), bounding_rectangles[i].br(), Scalar(0, 255, 0), 2, 8, 0);
    }

    // Make the color windows appear & return the bounding rectangles
    imshow(color_converter(color).c_str(), output);
    return bounding_rectangles;
}

// FILTER COLOR FUNCTION
// filters the given image by the specified color 
// and does a bitflip to make blob tracking easier later on
Mat ColorReader::threshold_image(Mat image, int color) 
{
    Mat filtered_image;
    Mat1b filter1, filter2;

    switch(color) 
    {
        case RED:
           inRange(image, Scalar(0, 130, 65), Scalar(7, 200, 95 + 25), filter1); 
           inRange(image, Scalar(170, 130, 65), Scalar(180, 200, 95 + 25), filter2);
           filtered_image = filter1 | filter2; 
           break;
    
        case ORANGE:
           inRange(image, Scalar(4, 30, 90), Scalar(12, 255, 255), filtered_image); 
           break;
    
        case YELLOW:
           inRange(image, Scalar(12, 10, 60 - 20), Scalar(38, 255, 150 + 20), filtered_image); 
           break;
    
        case GREEN:
           inRange(image, Scalar(60, 100, 0), Scalar(80, 255, 255), filtered_image); 
           break;
    
        case BLUE:
            inRange(image, Scalar(100, 100, 0), Scalar(115, 255, 255), filtered_image); 
            break;
    }
  
    bitwise_not(filtered_image, filtered_image);

    return filtered_image;
}

// EXTRACT COLOR FUNCTION
// extracts the colors at the given offset of an image
int ColorReader::extract_color(Mat& image, int x_offset, int y_offset) 
{
    Rect outline(x_offset - 2, y_offset - 2, SIDE + 2, SIDE + 2);
    rectangle(image, outline, Scalar(255, 0, 0), 2);

    Rect sample(x_offset, y_offset, SIDE, SIDE);
    Mat rgb_image = image(sample);
    Mat hsv_image;
    cvtColor(rgb_image, hsv_image, CV_BGR2HSV);
   
    int color = -1;
    float hue = 0;
    float saturation = 0;
    float value = 0;

    for (int i = 0; i < SIDE; i++) 
    {
        for (int j = 0; j < SIDE; j++) 
        {
            Vec3b hsv = hsv_image.at<Vec3b>(0, 0);
            hue+= hsv.val[0]; 
            saturation+= hsv.val[1];
            value+= hsv.val[2];
        }
    }
   
    hue = (hue / (SIDE * SIDE)) * 2;
    saturation = (saturation / (SIDE * SIDE)) / 255.0 * 100;
    value = (value / (SIDE * SIDE)) / 255.0 * 100;
    //ROS_INFO("H: %.0f, S: %.0f, V: %.0f \n", hue, saturation, value);

    if (saturation < 30) color = WHITE;
    else 
    {
        if (((hue >= 0 && hue < 14) || hue >= 249) && value < 95) color = RED;
        else if ((hue >= 8 && hue <= 27) && value > 95) color = ORANGE;
        else if (hue >= 24 && hue <= 76) color = YELLOW;
        else if (hue >= 120 && hue <= 160) color = GREEN;
        else if (hue >= 200 && hue <= 230) color = BLUE;
    }

    return color;
}

// PRINT COLORS FUNCTION
// prints a vector of colors nicely
void ColorReader::print_colors() 
{
    string color_string = "";

    for (int i = 0; i < 3; i++) 
    {     
        for (int j = 0; j < 3; j++)
        {
            color_string = color_string + color_converter(this->colors[i][j]) + "\t";
        }
        
        color_string+= "\n";
    }
    
    cout << color_string << endl;
}

// COLOR CONVERTER FUNCTION
// helper function to convert color number 
// representations into strings woot
string color_converter(int color) 
{
    string s;

    switch (color) 
    {
        case RED:
            s = "red";
            break;

        case ORANGE:
            s = "orange";
            break;

        case YELLOW:
            s = "yellow";
            break;

        case GREEN:
            s = "green";
            break;

        case BLUE:
            s = "blue";
            break;

        case WHITE:
            s = "white";
            break;
    }

    return s;
}

////////////////////////////////////////////////////////////////
