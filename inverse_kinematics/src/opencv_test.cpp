#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    // Set the image
    Mat image;
    image = imread("/home/csrobot/Desktop/shapes.jpg", 1);

    // Make sure the image has data
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    // Display the image
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);

    // Closes image when 
    waitKey(0);

    return 0;
}
