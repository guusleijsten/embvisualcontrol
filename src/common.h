#ifndef _COMMON_H_
#define _COMMON_H_
#include <sstream>
#include <String>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>


using namespace cv;
RNG rng(12345);
//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
extern const int FRAME_WIDTH = 640;
extern const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=10;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 24*24;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1;
//names that will appear at the top of each window
const String windowName = "Original Image";
const String windowName1 = "HSV Image";
const String windowName2 = "yellow";
const String windowName3 = "blue";
const String windowName4 = "red";
//const String windowName5 = "Hough";
const String windowName5 = "Hough";
const String trackbarWindowName = "Trackbars";

#endif