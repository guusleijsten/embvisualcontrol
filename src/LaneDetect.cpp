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
#include <cmath>
#include "string"

#define PI 3.1415926

#define KEY_SPACE 32
#define KEY_ENTER 13

//using namespace raspicam;
using namespace cv;
using namespace std;
const String windowName5 = "Hough";


int trafficSignClassifier(cv::Mat image) {

    int houghVote = 200;

    if (! image.data) {
        std::cout << "Error to read the image. Check ''cv::imread'' function of OpenCV" << std::endl;
        return -1;
    }

    Mat sImage;
    // cvtColor(image, gray, CV_RGB2GRAY);

    cv::resize(image, sImage, cv::Size(image.cols * 0.5,image.rows * 0.5), 0, 0, CV_INTER_LINEAR);
    //std::cout << "width: " << sImage.cols << ", height: " << sImage.rows << "\n";
    vector<string> codes;
    Mat corners;

    // Canny algorithm
    Mat contours;
    Canny(sImage, contours, 50, 250);
    Mat contoursInv;
    threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);

    // Display Canny image
    /*imwrite("../contours.bmp", contoursInv);*/

    /*
     Hough tranform for line detection with feedback
     Increase by 25 for the next frame if we found some lines.
     This is so we don't miss other lines that may crop up in the next frame
     but at the same time we don't want to start the feed back loop from scratch.
     */
    std::vector<Vec2f> lines;
    if (houghVote < 1 or lines.size() > 2) { // we lost all lines. reset
        houghVote = 200;
    } else {
        houghVote += 25;
    }
    while (lines.size() < 5 && houghVote > 0) {
        HoughLines(contours, lines, 1,PI / 4, houghVote);
        //HoughLines(contours, lines, 1, 145*PI / 180, houghVote);
        houghVote -= 5;
    }
    //std::cout << "Houghvote: " << houghVote << "\n";
    Mat result(sImage.size(), CV_8U, Scalar(255));
    sImage.copyTo(result);

    // Draw the lines
    std::vector<Vec2f>::const_iterator it = lines.begin();
    Mat hough(sImage.size(), CV_8U, Scalar(0));
    
    float minDist = FLT_MAX, maxDist = FLT_MIN;
    while (it != lines.end()) {

        float rho = (*it)[0]; // first element is distance rho
        float theta = (*it)[1]; // second element is angle theta

       if ((theta > 0.2 && theta < 1.4) || (theta > 1.6 && theta < 3.1)) { // filter to remove vertical and horizontal lines

           // point of intersection of the line with first row
           Point pt1(rho / cos(theta), 0);
           // point of intersection of the line with last row
           Point pt2((rho - result.rows * sin(theta)) / cos(theta), result.rows);
            // draw a white line
            line(result, pt1, pt2, Scalar(255), 8);
            line(hough, pt1, pt2, Scalar(255), 8);
            //std::cout << " line: rho:"<< rho <<" theta:" << theta << "\n";
            float dist = rho * cos(.5*PI - theta);

            if (dist > maxDist) {
                maxDist = dist;
            }
            if (dist < minDist) {
                minDist = dist;
            }
        }

        ++it;
    }

    std::cout << "min: " << minDist << "  ";
    std::cout << "max: " << maxDist << "\n";
    imshow(windowName5,result );

    if (maxDist > 0.4 && maxDist < 0.7) {
        return 1;//Straight
    }
    if (maxDist > 0.75) {
        return 2;//Right
    }
    if (maxDist < 0.25) {
        return 3;//Left
    }


    return 0;
}
