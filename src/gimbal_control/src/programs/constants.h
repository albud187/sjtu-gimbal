#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <iostream>


//ui camera params
int IMAGE_HEIGHT = 800;
int IMAGE_WIDTH = 800;

int IMG_CENTER_H = IMAGE_HEIGHT/2;
int IMG_CENTER_W = IMAGE_WIDTH/2;
int VEL_LINE_THICKNESS = 6;
int UI_SQ_SIDE_LENTH = 125;

float FOCAL_LEN = 232.1136;

//radians
float CAMERA_FOV = 2.09;
float MAX_ROT_RATE_1 = 0.01;
float MAX_ROT_RATE_2 = 0.01;

// rad/s
float MAX_GIMBAL_VEL = 0.2;
float TEST_GIMBAL_VEL = 0.2;
#endif