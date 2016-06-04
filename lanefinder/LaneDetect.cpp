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
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>
#include <raspicam/raspicamtypes.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include "linefinder.h"
#include "string"

#define PI 3.1415926

#define KEY_SPACE 32
#define KEY_ENTER 13

using namespace raspicam;
using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {

    int houghVote = 200;

    RaspiCam_Cv cam;

    //VideoCapture capture(arg);

    //if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
    //    	{capture.open(atoi(arg.c_str()));}

    cam.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    if (!cam.open())
      return 1;

    
    // Read the input image - convert char* to string
    //std::string input_filename(argv[1]);
    
    //if (input_filename == null_ptr)
      //  return 1;
    // Read the input image
    //cv::Mat image = cv::imread(input_filename);
    
    // Check that the image has been opened
    /*if (! image.data) {
        std::cout << "Error to read the image. Check ''cv::imread'' function of OpenCV" << std::endl;
        return -1;
    }*/
/**
    int ex = static_cast<int>(cam.get(CV_CAP_PROP_FOURCC));
    double dWidth = cam.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cam.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    std::cout << "Frame Size = " << dWidth << "x" << dHeight << std::endl;

    Size frameSize(static_cast<int> (dWidth), static_cast<int> (dHeight));
*/
    //VideoWriter oVideoWriter("LaneDetection.avi", CV_FOURCC('P', 'I', 'M', '1'), 20, frameSize, true); //initialize the VideoWriter object
    //TODO: fix video writer
    //VideoWriter outputVideo;
    //outputVideo.open("LaneDetection.avi", ex, cam.get(CV_CAP_PROP_FPS), frameSize, true);

    //if (!outputVideo.isOpened())
    //    return 1;
    Mat image;
    //int n = 1;
    while (1) {
	//    n--;
    cam.grab();
    cam.retrieve(image);
    //TODO: fix raw video writer
    //outputVideo.write(image);

    //if (image.empty())
    //break;
    Mat gray, sImage;
    cvtColor(image, gray, CV_RGB2GRAY);

    cv::resize(image, sImage, cv::Size(image.cols * 0.5,image.rows * 0.5), 0, 0, CV_INTER_LINEAR);
    
    vector<string> codes;
    Mat corners;
    //findDataMatrix(gray, codes, corners);
    //drawDataMatrixCodes(image, codes, corners);

    Rect roi(0, .7*sImage.rows, sImage.cols - 1, .3*sImage.rows-1); // set the ROI for the image
    Mat imgROI = sImage(roi);
    // Display the image
    imwrite("../images/original.bmp", imgROI);
    namedWindow( "Region of Interrest", WINDOW_AUTOSIZE ); // Create a window for display
    imshow( "Region of Interrest", imgROI );

    // Canny algorithm
    Mat contours;
    Canny(imgROI, contours, 50, 250);
    Mat contoursInv;
    threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);

    // Display Canny image
    imwrite("../images/contours.bmp", contoursInv);

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
    std::cout << "Houghvote: " << houghVote << "\n";
    Mat result(imgROI.size(), CV_8U, Scalar(255));
    imgROI.copyTo(result);

    // Draw the lines
    std::vector<Vec2f>::const_iterator it = lines.begin();
    Mat hough(imgROI.size(), CV_8U, Scalar(0));
    while (it != lines.end()) {

        float rho = (*it)[0]; // first element is distance rho
        float theta = (*it)[1]; // second element is angle theta

        //std::cout << " line: rho:"<< rho <<" theta:" << theta << "\n";
        //if ((theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66)) { // filter to remove vertical and horizontal lines
            //std::cout << " filter pass\n";
            // point of intersection of the line with first row
            Point pt1(rho / cos(theta), 0);
            // point of intersection of the line with last row
            Point pt2((rho - result.rows * sin(theta)) / cos(theta), result.rows);
            // draw a white line
            line(result, pt1, pt2, Scalar(255), 8);
            line(hough, pt1, pt2, Scalar(255), 8);
        //} else {
        //    std::cout << " filter fail\n";
        //}

        std::cout << "rho: " << rho << ", theta: " << theta << "\n";
        ++it;
    }
    // Display the detected line image
    imwrite("../images/hough.bmp", result);
    namedWindow( "Hough", WINDOW_AUTOSIZE ); // Create a window for display
    imshow( "Hough", hough );
    // Create LineFinder instance
    LineFinder ld;

    // Set probabilistic Hough parameters
    ld.setLineLengthAndGap(40, 5);
    ld.setMinVote(4);


    // Detect lines
    std::vector<Vec4i> li = ld.findLines(contours);
    Mat houghP(imgROI.size(), CV_8U, Scalar(0));
    ld.setShift(0);
    ld.drawDetectedLines(houghP);
    
    std::cout << "First Hough" << "\n";

    imwrite("../images/houghP.bmp", houghP);

    // bitwise AND of the two hough images
    bitwise_and(houghP, hough, houghP);
    Mat houghPinv(imgROI.size(), CV_8U, Scalar(0));

    threshold(houghP, houghPinv, 150, 255, THRESH_BINARY_INV); // threshold and invert to black lines

    imwrite("../images/houghPinv.bmp", houghPinv);

    Mat mm(imgROI.size(), CV_8U, Scalar(0));
    imwrite("../images/houghPinv.bmp", mm);

    Canny(houghPinv, contours, 100, 350);
    li = ld.findLines(contours);
    // Display Canny image
    namedWindow( "Hough contours", WINDOW_AUTOSIZE ); // Create a window for display
    imshow( "Hough contours", contours );

    // TODO: Check color within contour for match with lane
    imwrite("../images/Countours2.bmp", contours);

    //Mat result(imgROI.size(), CV_8U, Scalar(255));

    // Set probabilistic Hough parameters
    ld.setLineLengthAndGap(5, 2);
    ld.setMinVote(1);
    ld.setShift(.7*sImage.rows);
    ld.drawDetectedLines(sImage);

    namedWindow( "Prob Hough", WINDOW_AUTOSIZE ); // Create a window for display
    imwrite("../images/processed.bmp", sImage);
    
    int selected = 0, prev = 0;
    while(selected < ld.numberLines()) {
    
        ld.drawSelectedLine(sImage, selected, prev);
        
        imshow( "Prob Hough", sImage );
        int key = waitKey(0);
        //Act upon key press SPACE/ENTER
        if (selected > 0 && key == KEY_ENTER) {
            prev = selected--;
            continue;
        } else if (key == KEY_SPACE) {
            prev=selected++;
            continue; 
        } else if (key == KEY_ENTER) {
            continue;
        }
        break;
    }

    std::stringstream stream;
    stream << "Lines Segments: " << lines.size();
    }
    cam.release();
    return 0;
}
