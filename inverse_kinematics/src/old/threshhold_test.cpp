////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    // Open the image
    Mat imgOriginal = imread("/home/csrobot/Desktop/frame0000.jpg", 1);
    if (!imgOriginal.data)
    {
        cout << "No image data" << endl;
        return -1;
    }

    // Create a window called "Control"
    namedWindow("Control", CV_WINDOW_AUTOSIZE); 

    // Blue high lows
    int iLowH = 205 / 2;
    int iHighH = 225 / 2;
    int iLowS = 0; 
    int iHighS = 255;
    int iLowV = 0;
    int iHighV = 255;

    // Convert the captured frame from BGR to HSV
    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); 
 
    // Threshold the image
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 
      
    // Morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 

    // Morphological closing (fill small holes in the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    // Display the images
    imshow("Thresholded Image", imgThresholded); 
    imshow("Original", imgOriginal); 
    waitKey(0);

    return 0;
}
