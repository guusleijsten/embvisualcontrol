/* File : jpeg.h, header for all jpeg code */
/* Author: Pierre Guerrier, march 1998     */
/*                                         */
/* 19/01/99  Edited by Koen van Eijk       */

/* Leave structures in memory,output something and dump core in the event
   of a failure: */
#ifndef LANEDETECT_H
#define LANEDETECT_H

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int trafficSignClassifier(cv::Mat image);
#endif