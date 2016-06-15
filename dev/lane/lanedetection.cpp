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
#include "string"
#include <iostream>
#include "lanedetection.h"

#define PI 3.1415926
#define DELAY 4000
#define RESIZE_PROP 0.3
#define ROI_PROP 0.48
#define HOUGH_VOTE 225
#define BLUR_PAR 15
#define MIN_HOUGH_LINES 5

/// Global Variables
int KERNEL_LENGTH = 15;

int houghVote = 200;

using namespace cv;
using namespace std;

static string WINDOW_NAME = "Lane Detection step-by-step";

int getResponse(Mat image) {

    init();

    cv::Mat src = preprocess(image);

	//showImg(src);

    cv::Mat roi = clip(src);

    std::vector<Vec2f> lines = process(roi);
    Mat hough(roi.size(), CV_8U, Scalar(0));
    std::vector<Vec2f>::const_iterator it = lines.begin();
    
    Vec2f minLine, maxLine;
    minLine[1] = 10;
    maxLine[1] = 0;

    //Cluster hough lines on Row
    
    while (it != lines.end()) {

        float rho = (*it)[0]; // first element is distance rho
        float theta = (*it)[1]; // second element is angle theta
        
        //TODO: Get line minimum/maximimum theta, take difference in thetas, subtract from max. Same for rho. This gives reference line.
        if (((theta*180)/PI) > 105 || rho > 0)
        //if (rho > -100 && rho < 0)
        {
            if (minLine[1] > theta)
                minLine = *it;
            if (maxLine[1] < theta)
                maxLine = *it;
            // point of intersection of the line with first row
            Point pt1(rho / cos(theta), 0);
            // point of intersection of the line with last row
            Point pt2((rho - src.rows * sin(theta)) / cos(theta), src.rows);
            // draw a white line
            line(hough, pt1, pt2, Scalar(255));

            std::cout << "rho: " << rho << ", theta: " << ((theta*180)/PI) << "\n";
            //std::cout << " (" << pt1.x << ", " << pt1.y << ")->(" << pt2.x << "," << pt2.y << ")\n";
            //std::cout << " (" << pt1 << ", " << pt2 << ")\n";
        }
        ++it;
    }
    bool ret = true;
    if (maxLine[1] < .5*PI) {
        std::cout << "No RIGHT lane\n";
        ret = false;
    }
    if (minLine[1] > .5*PI) {
        std::cout << "No LEFT lane\n";
        ret = false;
    }

    //Point of intersection of the line with first row
    float x_mid = (maxLine[0] / cos(maxLine[1])) + (minLine[0] / cos(minLine[1]));
    
    std::cout << "vanish point " << x_mid/2 << "\n";
    Point pt1(x_mid/2, 0);
    
    //Point of intersection of the line with last row
    Point pt2(src.cols/2, src.rows);
    line(hough, pt1, pt2, Scalar(255), 5);
    
    //std::cout << "Max theta " << ((maxLine[1]*180)/PI) << "\n";
    //std::cout << "Min theta " << ((minLine[1]*180)/PI) << "\n";
    // Display the detected line image
    showImg(hough);

    return ret;
}

cv::Mat preprocess(cv::Mat src)
{
    cv::Mat resized, grayScaled;
    resize(src, resized, cv::Size(src.cols * RESIZE_PROP, src.rows * RESIZE_PROP), 
        0, 0, CV_INTER_LINEAR);
    cvtColor( resized, grayScaled, CV_BGR2GRAY );
    return grayScaled;
}

cv::Mat clip(cv::Mat src)
{
    Rect roi(0, (1-ROI_PROP)*src.rows, src.cols - 1, ROI_PROP*src.rows-1); // set the ROI for the image

    return src(roi);
}

std::vector<Vec2f> process(cv::Mat src)
{

    //cv::Mat mod;
    //equalizeHist( src, mod );
    //showImg(mod);

    cv::Mat blr;
    //GaussianBlur( mod, blr, Size( BLUR_PAR, BLUR_PAR ), 0, 0 );
    medianBlur(src, blr, BLUR_PAR);
    //showImg(blr);

    Mat contours;
    Canny(blr, contours, 50, 250);

    //showImg(contours);
    
    Mat contoursInv;
    threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);
    
    std::vector<Vec2f> lines;

    while (lines.size() < MIN_HOUGH_LINES && houghVote > 0) {
        HoughLines(contours, lines, 1, PI / 180, houghVote);
        houghVote -= 25;
    }
    //HoughLines(contours, lines, 1, PI / 180, HOUGH_VOTE);

    return lines;
}

void init(void)
{
    namedWindow( WINDOW_NAME, CV_WINDOW_AUTOSIZE );
}

void showImg(cv::Mat img)
{
    imshow( WINDOW_NAME, img );
    waitKey(0);
}
