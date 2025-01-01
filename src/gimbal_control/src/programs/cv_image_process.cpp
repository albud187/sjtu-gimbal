#include <mutex>
#include <thread>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "cv_image_process.h"

//snapshot
cv::Mat snapshot(cv::Mat src_img, cv::Point p1, cv::Point p2){
    int roi_top_left_x = p1.x;
    int roi_top_left_y = p1.y;
    int roi_width = p2.x - p1.x;
    int roi_height = p2.y - p1.y;

    cv::Rect roi(roi_top_left_x, roi_top_left_y, roi_width, roi_height);

    cv::Mat croppedImage = src_img(roi);

    return croppedImage;
}

cv::Point findTemplate(cv::Mat cv_ptr_image, cv::Mat template_image) {

    
    cv::Mat result;
    int result_cols = cv_ptr_image.cols - template_image.cols + 1;
    int result_rows = cv_ptr_image.rows - template_image.rows + 1;
    result.create(result_rows, result_cols, CV_32F);

    cv::matchTemplate(cv_ptr_image, template_image, result, cv::TM_CCOEFF_NORMED);
    cv::Point max_loc;
    double min_val, max_val;
    cv::minMaxLoc(result, &min_val, &max_val, nullptr, &max_loc);
    max_loc.x += template_image.cols / 2;
    max_loc.y += template_image.rows / 2;

    return max_loc;
}

cv::Mat edge_detect(cv::Mat cv_ptr_image){
    cv::Mat src_gray;
    cv::Mat dst, detected_edges;
    int lowThreshold = 25;
    int ratio = 3;
    int kernel_size = 3; 
    dst.create(cv_ptr_image.size(), cv_ptr_image.type());
    cv::cvtColor(cv_ptr_image, src_gray, cv::COLOR_BGR2GRAY);
    
    cv::Canny(src_gray, detected_edges, lowThreshold, lowThreshold * ratio, kernel_size);
    dst = cv::Scalar::all(0);
    cv_ptr_image.copyTo(dst, detected_edges);
    return dst;
}