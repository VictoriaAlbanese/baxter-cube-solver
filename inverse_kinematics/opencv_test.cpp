#include <stdio.h>
#include <opencv2/opencv.hpp>

#define SIZE 200
#define SIDE 5
#define CENTER_O ((SIZE / 2) - (SIDE / 2))
#define LEFT_O (CENTER_O - 32)
#define RIGHT_O (CENTER_O + 32)
#define MIDDLE_O CENTER_O - 2
#define TOP_O LEFT_O
#define BOTTOM_O RIGHT_O

using namespace cv;

void get_color(Mat img, int x_offset, int y_offset); 

int main(int argc, char** argv )
{
    // Set the image
    Mat image;
    image = imread("/home/csrobot/Desktop/frame0000.jpg", 1);

    // Make sure the image has data
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }

    // Crop the image
    Rect rect(618, 342, 120, 120);
    Mat cropped = image(rect);

    // Make the image bigger
    Mat bigger;
    resize(cropped, bigger, Size(SIZE, SIZE));
   
    // Get the color of each cubie
    get_color(bigger, LEFT_O, TOP_O);
    get_color(bigger, CENTER_O, TOP_O);
    get_color(bigger, RIGHT_O, TOP_O);
    get_color(bigger, LEFT_O, MIDDLE_O);
    get_color(bigger, CENTER_O, MIDDLE_O);
    get_color(bigger, RIGHT_O, MIDDLE_O);
    get_color(bigger, LEFT_O, BOTTOM_O);
    get_color(bigger, CENTER_O, BOTTOM_O);
    get_color(bigger, RIGHT_O, BOTTOM_O);
    
    // Display the image
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", bigger);

    // Closes image when 
    waitKey(0);

    return 0;
}

void get_color(Mat img, int x_offset, int y_offset) 
{
    Rect outline(x_offset - 2, y_offset - 2, SIDE + 2, SIDE + 2);
    rectangle(img, outline, Scalar(255, 0, 0), 2);

    Rect sample(x_offset, y_offset, SIDE, SIDE);
    Mat rgb_image = img(sample);
    Mat hsv_image;
    cvtColor(rgb_image, hsv_image, CV_BGR2HSV);
   
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

    printf("H: %.0f, S: %.0f, V: %.0f ", hue, saturation, value);

    if (saturation < 20) printf("\t -- white\n");
    else 
    {
        if (hue < 5 || hue > 355) printf("\t -- red\n");
        else if (hue < 40) printf("\t -- orange\n");
        else if (hue < 60) printf("\t -- yellow\n");
        else if (hue < 150) printf("\t -- green\n");
        else if (hue < 255) printf("\t -- blue\n");
    }
}
