#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <mutex>
#include <thread>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "programs/constants.h"
#include "programs/cv_image_process.h"

class CameraCVNode : public rclcpp::Node
{
public:

    CameraCVNode() : Node("camera_cv_node"){
        //std::string ns = this->get_namespace();
        std::string ns = "sjtu_drone";
        std::string T_cam_img = "/front/image_raw";
        std::string T_cmd_vel = ns + "/cmd_vel";
        std::string T_target = ns + "/target_pixel";
        std::string T_flight_mode = ns +"/mode";
        cam_ui_sub = this->create_subscription<sensor_msgs::msg::Image>(
            T_cam_img, 10, std::bind(&CameraCVNode::camera_image_cb, this, std::placeholders::_1));

        cam_track_sub = this->create_subscription<sensor_msgs::msg::Image>(
            T_cam_img, 10, std::bind(&CameraCVNode::camera_track_cb, this, std::placeholders::_1));
    
        vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            T_cmd_vel, 10, std::bind(&CameraCVNode::vel_cb, this, std::placeholders::_1));
    
        flight_mode_sub = this->create_subscription<std_msgs::msg::String>(
            T_flight_mode, 10, std::bind(&CameraCVNode::mode_cb, this, std::placeholders::_1));

        target_reporter = this->create_publisher<geometry_msgs::msg::Vector3>(T_target, 120);
    }

private:
    std::mutex mutex_;
    std::string image_tile = "/front/image_raw";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_ui_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_track_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_mode_sub;
    
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr target_reporter;

    sensor_msgs::msg::Image camera_image;
    geometry_msgs::msg::Twist cmd_vel;
    std_msgs::msg::String flt_mode;
    geometry_msgs::msg::Vector3 pixel_target;
   
    cv::Mat target_image;
    int p0x = IMAGE_WIDTH*0.15;
    int p1x = IMAGE_WIDTH*0.85;
    int p01y = IMAGE_HEIGHT/2;
    int square_blue = 255;
    int square_red = 0;
    int square_green = 0;
    
    int roi_center_x = IMG_CENTER_W+1;
    int roi_center_y = IMG_CENTER_H+1;

    int roi_len = UI_SQ_SIDE_LENTH/2;
    
    void camera_image_cb(const sensor_msgs::msg::Image::SharedPtr msg) {

        cv::Point p0(p0x, p01y);
        cv::Point p1(p1x, p01y);
        cv::Scalar square_color(square_blue, square_green, square_red);
        cv::Scalar line_color(0,255,0);

        int base_top_left_x = roi_center_x - roi_len;
        int base_top_left_y = roi_center_y - roi_len;
        cv::Point roi_top_left(base_top_left_x, base_top_left_y);
        cv::Point roi_bot_right(base_top_left_x + 2*roi_len, base_top_left_y + 2*roi_len);
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::putText(cv_ptr->image, "MODE : "+ flt_mode.data, cv::Point(50,50), cv::FONT_HERSHEY_DUPLEX, 0.75, line_color, 1.5);
            cv::rectangle(cv_ptr->image, roi_top_left, roi_bot_right, square_color, 1);
            
            if (flt_mode.data=="TRACK"){
                target_image = snapshot(cv_ptr->image, roi_top_left, roi_bot_right);
                cv::imshow("cropped image", target_image);
                flt_mode.data = "TRACKING";
            }

            if (flt_mode.data=="MAN_FLT"){
                square_blue = 255;
                square_red = 0;
                square_green = 0;
                roi_center_x = IMG_CENTER_W+1;
                roi_center_y = IMG_CENTER_H+1;
                roi_len = UI_SQ_SIDE_LENTH/2;
            }

            int pvx_y = p01y - 5*cmd_vel.linear.x;
            int pvy_x = p0x - 5*cmd_vel.linear.y;
            int pvz_y = p01y - 10*cmd_vel.linear.z;
            int prz_x = p1x - 20*cmd_vel.angular.z;
            cv::Point p_vx(p0x, pvx_y);
            cv::Point p_vy(pvy_x, p01y);
            cv::Point p_vz(p1x, pvz_y);
            cv::Point p_rz(prz_x, p01y);

            cv::line(cv_ptr->image, p0, p_vx, line_color, VEL_LINE_THICKNESS, cv::LINE_8);
            cv::line(cv_ptr->image, p0, p_vy, line_color, VEL_LINE_THICKNESS, cv::LINE_8);
            cv::line(cv_ptr->image, p1, p_vz, line_color, VEL_LINE_THICKNESS, cv::LINE_8);
            cv::line(cv_ptr->image, p1, p_rz, line_color, VEL_LINE_THICKNESS, cv::LINE_8);
        
            cv::imshow(image_tile, cv_ptr->image);
            cv::Mat edge_img = edge_detect(cv_ptr->image);
            //cv::imshow("edge", edge_img);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void mode_cb(const std_msgs::msg::String::SharedPtr msg){
        flt_mode = *msg;
    }

    void vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
        cmd_vel = *msg;
    }

    void camera_track_cb(const sensor_msgs::msg::Image::SharedPtr msg){
        cv_bridge::CvImagePtr cv_track_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if (flt_mode.data=="TRACKING"){
            square_blue = 0;
            square_red = 255;
            square_green = 0;

            cv::Point target_roi = findTemplate(cv_track_ptr->image, target_image);
           
            roi_center_x = target_roi.x;
            roi_center_y = target_roi.y;

            int target_top_left_x = roi_center_x - UI_SQ_SIDE_LENTH/2;
            int target_top_left_y = roi_center_y - UI_SQ_SIDE_LENTH/2;
            cv::Point target_roi_top_left(target_top_left_x, target_top_left_y);
            cv::Point target_roi_bot_right(target_top_left_x + UI_SQ_SIDE_LENTH, target_top_left_y + UI_SQ_SIDE_LENTH);

            target_image = snapshot(cv_track_ptr->image, target_roi_top_left, target_roi_bot_right);
            cv::imshow("cropped image", target_image);

            pixel_target.x = roi_center_x;
            pixel_target.y = roi_center_y;
            target_reporter->publish(pixel_target);

            //publish the pixel coordinate
            
        }
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<CameraCVNode>();

    rclcpp::spin(rclcppNode);

    rclcpp::shutdown();
    return 0;
}