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

#define PI 3.1415926
#define DELAY 4000

/// Global Variables
int KERNEL_LENGTH = 13;

using namespace cv;
using namespace std;

string window_name = "Smoothing Demo";

/// Function headers
int display_caption( const string caption );
int display_dst( int delay );

int main(int argc, char* argv[]) {

	int houghVote = 200;
    
    //Mat fullImage = imread("../../img/img3.png");
    //Mat image;
    //resize(fullImage, image, cv::Size(fullImage.cols * 0.3,fullImage.rows * 0.3), 0, 0, CV_INTER_LINEAR);


	// // HSV
 //    Mat imageHSV;
 //    cvtColor(image, imageHSV, CV_BGR2HSV);
    
 //    imshow( "Debug", imageHSV );
 //    waitKey(0);

	// // Canny algorithm
 //    Mat contours;
 //    Canny(image, contours, 50, 250);
 //    Mat contoursInv;
 //    threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);
 //    imshow( "Debug", contours );
 //    waitKey(0);
 //    imshow( "Debug", contoursInv );
 //    waitKey(0);
	namedWindow( window_name, CV_WINDOW_AUTOSIZE );

	Mat image = imread("../../img/img3.png");
	/// Load the source image

	Mat src;
	resize(image, src, cv::Size(image.cols * 0.3,image.rows * 0.3), 0, 0, CV_INTER_LINEAR);

	imshow(window_name, src);
    waitKey(DELAY);

    Rect roi(0, .55*src.rows, src.cols - 1, .45*src.rows-1); // set the ROI for the image

    Mat imgROI = src(roi);
    imshow(window_name, imgROI);
    waitKey(DELAY);

    //Median blur
    Mat blr;
	medianBlur(imgROI, blr, KERNEL_LENGTH);
	imshow(window_name, blr);
	waitKey(DELAY);

	// //HSV
	// Mat hsv;
 // 	cvtColor(blr, hsv, CV_BGR2HSV);
 //    imshow(window_name, hsv);
	// waitKey(DELAY);

    // Canny algorithm
    Mat contours;
    Canny(blr, contours, 50, 250);
    Mat contoursInv;
    threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);
    //imshow( window_name, contours );
    //waitKey(0);
    imshow( window_name, contoursInv );
    waitKey(DELAY);

    //Hough
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
        HoughLines(contours, lines, 1, PI / 180, houghVote);
        houghVote -= 5;
    }

    // Draw the lines
    std::vector<Vec2f>::const_iterator it = lines.begin();
    Mat hough(imgROI.size(), CV_8U, Scalar(0));
    std::cout << "Sizeout " << imgROI.size() << "\n";
    while (it != lines.end()) {

        float rho = (*it)[0]; // first element is distance rho
        float theta = (*it)[1]; // second element is angle theta
        
        //TODO: modify code here!
        {
            // point of intersection of the line with first row
            Point pt1(rho / cos(theta), 0);
            // point of intersection of the line with last row
            Point pt2((rho - src.rows * sin(theta)) / cos(theta), src.rows);
            // draw a white line
            line(hough, pt1, pt2, Scalar(255), 8);
        
    
        	//std::cout << "rho: " << rho << ", theta: " << ((theta*180)/PI) << "\n";
        	//std::cout << " (" << pt1.x << ", " << pt1.y << ")->(" << pt2.x << "," << pt2.y << ")\n";
        	std::cout << " (" << pt1 << ", " << pt2 << ")\n";
        }
        ++it;
    }
    // Display the detected line image
    imshow( window_name, hough );
    waitKey(0);
	
	return 0;
 }
