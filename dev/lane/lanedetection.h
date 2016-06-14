#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "string"
#include <stdio.h>
#include <iostream>

struct Line
{
   float row;
   float theta;
};

void init(void);
void showImg(cv::Mat img);

cv::Mat clip(cv::Mat src);
cv::Mat preprocess(cv::Mat src);
std::vector<cv::Vec2f> process(cv::Mat src);

#endif