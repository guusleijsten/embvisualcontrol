
#include <sstream>
#include <String>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;

int main(int argc, char const *argv[])
{
	/* code */
	// if( argc != 2)
 //    {
 //     std::cout <<" Usage: display_image ImageToLoadAndDisplay" << std::endl;
 //     return -1;
 //    }

	std::vector< std::vector <Point> > contours; // Vector for storing contour
    std::vector< Vec4i > hierarchy;
	int count=0; 
  int startFrame=0;//2000;
  int frameCount=0;
    Mat image,HSV,thd_red;
    Mat dst(image.rows,image.cols,CV_8UC1,Scalar::all(0)); //create destination image
     image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file


  //video capture object to acquire webcam feed
  VideoCapture capture;
  // //open capture object at location zero (default location for webcam)
  capture.open("video3_conv.avi");
 //capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
 //capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

  while(1)
  {
    //store image to matrix
    capture.read(image);
    frameCount++;
    if(frameCount<startFrame)
      continue;

	cvtColor(image,HSV,COLOR_BGR2HSV);
	cv::inRange(HSV, Scalar(0, 150, 150), Scalar(20, 250, 240), thd_red);
  namedWindow( "Display window2", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window2", image );                   // Show our image inside it.


  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  imshow( "Display window", thd_red );                   // Show our image inside it.

  double refArea = 0;
	findContours(thd_red,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	for( int i = 0; i< contours.size(); i=hierarchy[i][0] ) // iterate through each contour.
       {   
        	 //Rect r= boundingRect(contours[i]);
       
            	// rectangle(image,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,0,255),3,8,0);
            	//count++;

              Moments moment = moments((cv::Mat)contours[i]);
              double area = moment.m00;
              if(area>24*24 && area<640*480 && area>refArea){

              std::vector<Point> approxCircles222;

              approxPolyDP(contours[i],approxCircles222,arcLength(Mat(contours[i]), true)*0.04, true);
              if (approxCircles222.size() >5  ){
              std::cout<<"Uturn";
              std::cout<<"vertex is " <<approxCircles222.size()
              <<std::endl;
              getchar();}
        	 }
          
          
      	}

	std::cout<<"Numeber of contour = "<<count<<std::endl;
  	waitKey(1);
}

	return 0;
}
