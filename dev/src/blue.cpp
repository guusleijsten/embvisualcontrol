#include <sstream>
#include <String>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <time.h>


using namespace cv;


void morphOps_blue(Mat &thresh)
{

  //create structuring element that will be used to "dilate" and "erode" image.
  //the element chosen here is a 3px by 3px rectangle
  // 

  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  //dilate with larger element so make sure object is nicely visible
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

  erode(thresh,thresh,erodeElement);
  erode(thresh,thresh,erodeElement);


  dilate(thresh,thresh,dilateElement);
  dilate(thresh,thresh,dilateElement);
}
int main(int argc, char const *argv[])
{
	std::vector< std::vector <Point> > contours; // Vector for storing contour
  std::vector< Vec4i > hierarchy;
	int count=0; 
  int startFrame=0;//2000;
 	int frameCount=0;
 	int x=0;
 	int y=0;
  Mat image,HSV,thd_red;
  Mat dst(image.rows,image.cols,CV_8UC1,Scalar::all(0)); //create destination image
     // image = imread("", CV_LOAD_IMAGE_COLOR);   // Read the file
	Moments moment;

  	//video capture object to acquire webcam feed
  	VideoCapture capture;
  	//open capture object at location zero (default location for webcam)
  	capture.open("video_new.h264");
  	// capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  	// capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

  	while(1)
  	{
  	  //store image to matrix
    	capture.read(image);
    	frameCount++;
    	if(frameCount<startFrame)
      		continue;

		  cvtColor(image,HSV,COLOR_BGR2HSV);
		  cv::inRange(HSV, Scalar(106, 100, 100), Scalar(115, 255, 255), thd_red);

  		namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
  		imshow( "Display window", thd_red );                   // Show our image inside it.

	  	namedWindow( "Display window2", WINDOW_AUTOSIZE );// Create a window for display.
  		imshow( "Display window2", image );                   // Show our image inside it.

      //morphOps_blue(thd_red);

  		double refArea = 0;
		findContours(thd_red,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );
		for( int i = 0; i< contours.size(); i=hierarchy[i][0] ) // iterate through each contour.
       {   
          std::vector<Point> approxCircles;
          approxPolyDP(contours[i], approxCircles, arcLength(Mat(contours[i]), true)*0.05, true);
          std::cout <<"vertex = "<< approxCircles.size() << std::endl ;
          if(approxCircles.size() >= 4){
        	 //Rect r= boundingRect(contours[i]);
        	//if(hierarchy[i][2]>0)
        	//{
            	// rectangle(image,Point(r.x,r.y), Point(r.x+r.width,r.y+r.height), Scalar(0,0,255),3,8,0);
            	count++;
                moment = moments((cv::Mat)contours[i]);
                double area = moment.m00;
                if(area>24*24 && area<640*480 && area>refArea)
              		{
                  		refArea = area;
        	 		}
			}
          else 
              count = 0;
      	}
      	double r = sqrt(refArea/3.14);
        if (r != 0){
		x = moment.m10/refArea;
		y = moment.m01/refArea;
		// Size size(300,300);//the dst image size,e.g.100x100
		// Mat dst;//dst image
		// Mat src;//src image
		// resize(src,dst,size);//resize image

		cv::Rect rect = cv::Rect(x-r, y-r-10, 2.4*r, 2.4*r);
		//cv::Rect rect = cv::Rect(x-r, y-r, 100, 180);
		std::cout<<"(x,y)= ("<<x<<", "<<y<<")"<<" r= "<<r <<std::endl;
		//getchar();

		if (rect.x + rect.width < thd_red.cols
		 &&	rect.y + rect.height < thd_red.rows
		 && rect.x > 0 && rect.y > 0) 
		{
			//get the target binary image
			cv::Mat crop = image(rect);
  		  cvtColor(crop,HSV,COLOR_BGR2HSV);
			  cv::inRange(HSV, Scalar(100, 100, 100), Scalar(130, 255, 255), crop);
			  namedWindow( "Display window4", WINDOW_AUTOSIZE );// Create a window for display.
  			imshow( "Display window4", crop );                   // Show our image inside it.
  			//getchar();
  			int s1 = cv::sum(crop(cv::Range(0,1.2*r), cv::Range(0,1.2*r)))[0];
  			int s2 = cv::sum(crop(cv::Range(1.2*r,2.4*r), cv::Range(0,1.2*r)))[0];
  			int s3 = cv::sum(crop(cv::Range(0, 1.2*r), cv::Range(1.2*r,2.4*r)))[0];
  			int s4 = cv::sum(crop(cv::Range(1.2*r,2.4*r), cv::Range(1.2*r,2.4*r)))[0];
  			//s1 = s1/255;
  			std::cout<<"s1= "<<s1<<std::endl;
  			std::cout<<"s2= "<<s2<<std::endl;
  			std::cout<<"s3= "<<s3<<std::endl;
  			std::cout<<"s4= "<<s4<<std::endl;

        int array[4] = {s1,s2,s3,s4};
        int array2[4] = {s1,s2,s3,s4};
        int temp = s4,temp2 = s1 ;

        for(int i=0;i<4;i++)
        {
          if(array[i]<temp)
          temp=array[i];
        }

        for(int i=0;i<4;i++)
        {
          if(array2[i]>temp2)
          temp2=array2[i];
        }

        if (temp2 == s2  && temp2 >0&& temp>0 && r > 17){
            std::cout<< "Left!"<< std::endl;
          }
        else{
          if (temp == s3 && temp2 == s4 && temp2 != 0 && temp != 0 && r > 17){
            std::cout<< "Right!"<< std::endl;
          }
          else{
            if (temp2  >1000&& temp>1000 && temp2/temp <1.2 )
            {
              std::cout<< "Straight!"<< std::endl;
              getchar();
            }
            else{
              getchar();
            }
          }
        }
      
          std::cout << "The smallest number is: " << temp << std::endl;
  		//getchar();
		}
  }

  		waitKey(1);
}

	return 0;
}

