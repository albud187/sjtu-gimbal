#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <mutex>
#include <thread>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

cv::Mat snapshot(cv::Mat src_img, cv::Point p1, cv::Point p2);

cv::Point findTemplate(cv::Mat cv_ptr_image, cv::Mat cropped_image);

cv::Mat edge_detect(cv::Mat cv_ptr_image);




#endif