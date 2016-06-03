//objectTrackingTutorial.cpp

//Written by  Kyle Hounslow 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.


#include "common.h"
#include <time.h>
#include "LaneDetect.h"
#include "math.h"
#include "stop.cpp"
//#include "drawObjectDetect.cpp"

using namespace cv;
#define PI 3.1415926


void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

void createTrackbars(){
	//create window for trackbars


    namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


}
// void drawObject(int x, int y,Mat &frame){

// 	//use some of the openCV drawing functions to draw crosshairs
// 	//on your tracked image!

//     //UPDATE:JUNE 18TH, 2013
//     //added 'if' and 'else' statements to prevent
//     //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)
//     //void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
// 	// cv::Rect(frame,(x-20,sqrt(x*x+y*y+800+40*(y-x))),(x+20,sqrt(x*x+y*y+800+40*(x-y))),Scalar(0,255,0),2,8);
// 	circle(frame,Point(x,y),50,Scalar(0,255,0),2);
//     if(y-25>0)
//     line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
//     else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
//     if(y+25<FRAME_HEIGHT)
//     line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
//     else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
//     if(x-25>0)

//     line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
//     else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
//     if(x+25<FRAME_WIDTH)
//     line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
//     else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

// 	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

// }
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	


}
void wait(int seconds)
{
	clock_t endwait;
	endwait = clock() + seconds * CLOCKS_PER_SEC;
	while (clock() < endwait) {}
}

void trackFiltered(Mat threshold, Mat &cameraFeed){

    int x = 0; int y = 0;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	std::vector< std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                	//compute the center of the contour
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true){

				//putText(cameraFeed,"Tracking Object",Point(0,80),2,1,Scalar(0,255,0),2);
				//draw object location on screen
                //rectangle(cameraFeed, x,y, Scalar(0,255,0));

				drawObject(x,y,cameraFeed);}


		}//else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

void trackBlue(Mat threshold, Mat &cameraFeed){
    int x = 0; int y = 0;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	std::vector< std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//double r = sqrt(area/3.14);

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                	//compute the center of the contour
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true){
				// double r = sqrt(refArea/3.14);

				 // Mat drawing = Mat::zeros( temp.size(), CV_8UC3 );
 				//  for( int i = 1*contours.size()/2; i< contours.size(); i++ )
   		// 		 {
     // 			  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
     //  			  drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     //  			  //approxPolyDP(drawing,drawing,0.20,0);
    	// 		  //Rect r= boundingRect(contours[i]); 
    	// 		 }

    			// cv::Rect rect = cv::Rect(x-r, y-r, 2*r, r);
    			//     int houghVote = 200;
    			//std::cout << "x" << (x*r) << "\n";
    			//std::cout << "y" << (y+.5*r) << "\n";
    			//std::cout << "width" << r << "\n";
    			//std::cout << "height" << r << "\n";
    			// int ret = -1;
    			// if (rect.x + rect.width < drawing.cols - 1 && rect.y + rect.height < drawing.rows - 1
    			// 	&& rect.x > 0 && rect.y > 0) {
    			// 	cv::Mat crop = drawing(rect);
	    		// 	 ret = trafficSignClassifier(crop);
	    			// switch(ret) {
	    			// 	case 1:
	    			// 		std::cout << "Straight!\n";
	    			// 		break;
    				// 	case 2:
	    			// 		std::cout << "Right!\n";
	    			// 		break;
    				// 	case 3:
	    			// 		std::cout << "Left!\n";
	    			// 		break;
    				// 	default:
	    			// 		std::cout << "Nope\n";
	    			// 		break;
   				//putText(cameraFeed,"Tracking Object",Point(0,80),2,1,Scalar(0,255,0),2);
				//draw object location on screen
                // cv::Rect(xMin,yMin,xMax-xMin,yMax-yMin);
                drawObject(x,y,cameraFeed);
   					}
		}//else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

// void trackStop(Mat threshold, Mat &cameraFeed){

//     int x = 0; int y = 0;
// 	Mat temp;
// 	threshold.copyTo(temp);
// 	//these two vectors needed for output of findContours
// 	std::vector< std::vector<Point> > contours;
// 	std::vector<Vec4i> hierarchy;
// 	//find contours of filtered image using openCV findContours function
// 	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

// 	//use moments method to find our filtered object
// 	double refArea = 0;
// 	bool objectFound = false;
// 	if (hierarchy.size() > 0) {
// 		int numObjects = hierarchy.size();
//         //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
//         if(numObjects<MAX_NUM_OBJECTS){
// 			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

// 				Moments moment = moments((cv::Mat)contours[index]);
// 				double area = moment.m00;

// 				//if the area is less than 20 px by 20px then it is probably just noise
// 				//if the area is the same as the 3/2 of the image size, probably just a bad filter
// 				//we only want the object with the largest area so we safe a reference area each
// 				//iteration and compare it to the area in the next iteration.
//                 if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
//                 	//compute the center of the contour
// 					x = moment.m10/area;
// 					y = moment.m01/area;
// 					objectFound = true;
// 					refArea = area;
// 				}else objectFound = false;
// 			}
// 			//let user know you found an object
// 			if(objectFound ==true){

// 				putText(cameraFeed,"Tracking Object",Point(0,80),2,1,Scalar(0,255,0),2);
// 				//draw object location on screen
//                 //rectangle(cameraFeed, x,y, Scalar(0,255,0));

// 				drawObject(x,y,cameraFeed);}


// 		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
// 	}
// }
// int main(int argc, char* argv[])
void ColorTracking()
{
  	
	//some boolean variables for different functionality within this
	//program
    bool trackObjects = true;
    bool useMorphOps = true;
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::Mat red_hue_image;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat thd_yellow;
	Mat thd_blue;
	Mat thd_red;
	//x and y values for the location of the object
	//create slider bars for HSV filtering
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open("test_video2.avi");
	// Mat image;
 //    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

 //    if(! image.data )                              // Check for invalid input
 //    {
 //        printf("Could not open or find the image\n");
 //        //cout <<  "Could not open or find the image" << std::endl ;
 //        return -1;
 //    }
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while(1){
	
		//store image to matrix

		capture.read(cameraFeed);
		 // Mat image;
       //  image = imread("stop.jpg");   // Read the file
         // cameraFeed = image;

		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix		  int iLowH = 166
  
		// inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
        cv::GaussianBlur(HSV, HSV, cv::Size(9, 9), 2, 2);
  	    cv::inRange(HSV, Scalar(20, 150, 50), Scalar(30, 255, 255), thd_yellow);
  	    cv::inRange(HSV, Scalar(110, 50, 50), Scalar(130, 255, 255), thd_blue) ;  		

  	    cv::inRange(HSV, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_red_hue_range);
 	    cv::inRange(HSV, Scalar(160, 100, 100), Scalar(179, 255, 255), upper_red_hue_range);
 	     
 	    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, thd_red);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if(useMorphOps){
		morphOps(thd_red);
		morphOps(thd_blue);
		morphOps(thd_yellow);}
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if(trackObjects){
			trackStop(thd_red,cameraFeed);
			trackBlue(thd_blue,cameraFeed);
			trackFiltered(thd_yellow,cameraFeed);}

		//show frames 
		imshow(windowName3,thd_blue);
		imshow(windowName4,thd_red);
		imshow(windowName2,thd_yellow);
		//imshow(windowName,cameraFeed);
		imshow(windowName1,HSV);
		

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(30);
	}

	//return 0;
}

