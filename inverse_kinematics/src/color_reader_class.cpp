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
vector<int> ColorReader::get_colors() 
{ 
    ros::Duration(5.0).sleep();
    this->print_colors();
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
    Rect rect(560, 330, 120, 120);
    Mat cropped = cv_ptr->image(rect);
    resize(cropped, cv_ptr->image, Size(SIZE, SIZE));
    Rect cube(30, 30, 80, 80);

    // Get the color of each cubie
    this->inspect_face(cv_ptr->image);

    // Update window
    imshow(CR_WINDOW_NAME, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    pub.publish(cv_ptr->toImageMsg());
}


// INSPECT FACE FUNCTION
// extracts the colors in a predetermined location 
// where the cube is supposed to be, getting a color 
// reading for each cubie & returning a vector of them
void ColorReader::inspect_face(Mat& image) 
{
    vector<int> cube_colors;
    cube_colors.push_back(extract_color(image, LEFT_O, TOP_O));
    cube_colors.push_back(extract_color(image, CENTER_O, TOP_O));
    cube_colors.push_back(extract_color(image, RIGHT_O, TOP_O));
    cube_colors.push_back(extract_color(image, LEFT_O, MIDDLE_O));
    cube_colors.push_back(extract_color(image, CENTER_O, MIDDLE_O));
    cube_colors.push_back(extract_color(image, RIGHT_O, MIDDLE_O));
    cube_colors.push_back(extract_color(image, LEFT_O, BOTTOM_O));
    cube_colors.push_back(extract_color(image, CENTER_O, BOTTOM_O));
    cube_colors.push_back(extract_color(image, RIGHT_O, BOTTOM_O));

    this->colors = cube_colors;
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
    //printf("H: %.0f, S: %.0f, V: %.0f \n", hue, saturation, value);

    if (saturation < 30) color = WHITE;
    else 
    {
        if (((hue >= 0 && hue < 14) || hue >= 249) && value < 88) color = RED;
        else if ((hue >= 2 && hue <= 27) && value > 88) color = ORANGE;
        else if (hue >= 32 && hue <= 62) color = YELLOW;
        else if (hue >= 120 && hue <= 150) color = GREEN;
        else if (hue >= 201 && hue <= 231) color = BLUE;
    }

    return color;
}

// PRINT COLORS
// prints a vector of colors nicely
void ColorReader::print_colors() 
{
    string color_string = "";

    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++)
        {
            if (this->colors[3 * i + j] == RED) color_string+= "red\t";
            else if (this->colors[3 * i + j] == ORANGE) color_string+= "orange\t";
            else if (this->colors[3 * i + j] == BLUE) color_string+= "blue\t";
            else if (this->colors[3 * i + j] == GREEN) color_string+= "green\t";
            else if (this->colors[3 * i + j] == YELLOW) color_string+= "yellow\t";
            else if (this->colors[3 * i + j] == WHITE) color_string+= "white\t";
        }
        
        color_string+= "\n";
    }
    
    cout << color_string << endl;
}

////////////////////////////////////////////////////////////////
