#include <stdio.h>
#include <opencv2/opencv.hpp>

#define SIZE 500

#define WHITE 0
#define YELLOW 1
#define RED 2
#define ORANGE 3
#define BLUE 4
#define GREEN 5

using namespace cv;
using std::cout;
using std::endl;

float toH(int input) { return input / 2.0; };
float toSV(int input) { return input * 255 / 100.0; };
void extract_color(Mat img, int color); 

int main(int argc, char** argv )
{
    // Set the image
    Mat image = imread("/home/csrobot/Desktop/frame0000.jpg", 1);
    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }
 
    // Crop the image
    Rect rect(620, 330, 120, 120);
    Mat cropped = image(rect);
    resize(cropped, image, Size(SIZE, SIZE));
  
    // Read the colors from the face of the image
    //extract_color(cropped, WHITE);
    //extract_color(cropped, YELLOW);
    //extract_color(cropped, RED);
    //extract_color(cropped, ORANGE);
    extract_color(cropped, GREEN);
    //extract_color(cropped, BLUE);

    // Closes image when 
    waitKey(0);

    return 0;
}

void extract_color(Mat img, int color) 
{
    // Convert the captured frame from BGR to HSV
    Mat imgHSV;
    cvtColor(img, imgHSV, COLOR_BGR2HSV); 
 
    // Threshold the image
    Mat imgThresholded;
    Scalar low, high;
    Scalar nlow, nhigh;

    switch(color) 
    {
        case WHITE:
           low = Scalar(0, 0, 0);
           high = Scalar(179, 25, 255);
           break;

        case YELLOW:
           low = Scalar(18, 0, 0);
           high = Scalar(28, 255, 255);
           break;

        case RED:
            low = Scalar(0, 0, 0);
            high = Scalar(7, 255, 88);
            nlow = Scalar(176, 0, 0);
            nhigh = Scalar(179, 255, 88);
            break;

        case ORANGE:
            low = Scalar(1, 0, 89);
            high = Scalar(11, 255, 255);
            break;

        case GREEN:
            low = Scalar(62, 0, 0);
            high = Scalar(72, 255, 255);
            break;

        case BLUE:
            low = Scalar(102, 0, 0);
            high = Scalar(112, 255, 255);
            break;
    }

    inRange(imgHSV, low, high, imgThresholded); 
    
    // Morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 

    // Morphological closing (fill small holes in the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    params.filterByArea = true;
    params.minArea = 200;
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByCircularity = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    // Get the blobs
    std::vector<KeyPoint> keypoints;
    SimpleBlobDetector detector(params);
    detector.detect(imgThresholded, keypoints);

    // Draw detected blobs as red circles.
    Mat imgKeypoints;
    drawKeypoints(imgThresholded, keypoints, imgKeypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Show blobs
    imshow("Original", img); 
    imshow("Keypoints", imgKeypoints);
}

