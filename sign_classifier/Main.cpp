/*------------------------------------------------------------------------------------------*\
 Lane Detection

 General idea and some code modified from:
 chapter 7 of Computer Vision Programming using the OpenCV Library.
 by Robert Laganiere, Packt Publishing, 2011.

 This program is free software; permission is hereby granted to use, copy, modify,
 and distribute this source code, or portions thereof, for any purpose, without fee,
 subject to the restriction that the copyright notice may not be removed
 or altered from any source or altered source distribution.
 The software is released on an as-is basis and without any warranties of any kind.
 In particular, the software is not guaranteed to be fault-tolerant or free from failure.
 The author disclaims all warranties with regard to this software, any use,
 and any consequent failure, is purely the responsibility of the user.

 Copyright (C) 2013 Jason Dorweiler, www.transistor.io


 Notes:

 Add up number on lines that are found within a threshold of a given rho,theta and
 use that to determine a score.  Only lines with a good enough score are kept.

 Calculation for the distance of the car from the center.  This should also determine
 if the road in turning.  We might not want to be in the center of the road for a turn.

 Several other parameters can be played with: min vote on houghp, line distance and gap.  Some
 type of feed back loop might be good to self tune these parameters.

 We are still finding the Road, i.e. both left and right lanes.  we Need to set it up to find the
 yellow divider line in the middle.

 Added filter on theta angle to reduce horizontal and vertical lines.

 Added image ROI to reduce false lines from things like trees/powerlines
 \*------------------------------------------------------------------------------------------*/

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/*
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>
#include <raspicam/raspicamtypes.h>
 */
#include <iostream>
#include <vector>
#include <stdio.h>
#include "string"
#include "LaneDetect.h"

#define PI 3.1415926

//using namespace raspicam;
using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {

    //RaspiCam_Cv cam;

    //VideoCapture capture(arg);

    //if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
    //    	{capture.open(atoi(arg.c_str()));}
/*
    cam.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    if (!cam.open())
    return 1;
*/
    
    // Read the input image - convert char* to string
    std::string input_filename(argv[1]);
    
    //if (input_filename == null_ptr)
      //  return 1;
    // Read the input image
    cv::Mat image = cv::imread(input_filename);
    
    // Check that the image has been opened
    if (! image.data) {
        std::cout << "Error to read the image. Check ''cv::imread'' function of OpenCV" << std::endl;
        return -1;
    }

    int ret = trafficSignClassifier(image);

    std::cout << "ret: " << ret << "\n";

    return 0;
}
