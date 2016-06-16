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
#include <stdio.h>
#include "String"
#include <iostream>
 #include <sstream>
#include "lanedetection.h"

int main(int argc, char *argv[]) {

    //init();
    using namespace cv;

  cv::Mat image ;

	cv::VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open("video_new.h264");
int i = 0;
    while(1){

	   capture.read(image);

        // cv::Mat image = cv::imread(argv[1]);
        if (i % 5 == 0)
            std::cout << getResponse(image) << "!";}
        i++;
    return 0;
}
