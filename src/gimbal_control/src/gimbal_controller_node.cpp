#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>
#include <thread>
#include <cmath>
#include <iostream>
#include "programs/constants.h"

class GimbalControlNode : public rclcpp::Node
{
public:

    GimbalControlNode() : Node("gimbal_control_node"){
        //std::string ns = this->get_namespace();
        std::string ns = "sjtu_drone";
       
        //subscribe to target_pixel
        std::string T_gimbal_angles = "/gimbal_angles";
        std::string T_target = ns + "/target_pixel";
        std::string T_flight_mode = ns +"/mode";
        std::string T_gimbal_step = "/gimbal_step";

        flight_mode_sub = this->create_subscription<std_msgs::msg::String>(
            T_flight_mode, 10, std::bind(&GimbalControlNode::mode_cb, this, std::placeholders::_1));
        
        target_pixel_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            T_target, 50, std::bind(&GimbalControlNode::target_pixel_cb, this, std::placeholders::_1));

        gimba_step_pub = this->create_publisher<geometry_msgs::msg::Vector3>(T_gimbal_step, 60);
        gimbal_angle_pub = this->create_publisher<geometry_msgs::msg::Vector3>(T_gimbal_angles, 60);
    
        gimbal_step_thread = std::thread(&GimbalControlNode::publish_gimbal_angles, this);

    
    }

    void join_gimbal_step_thread(){
        if (gimbal_step_thread.joinable()){
            gimbal_step_thread.join();
        }
    }

private:
    std::mutex mutex_;
    //subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_mode_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_pixel_sub;
    
    //publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_angle_pub;

    //messages
    geometry_msgs::msg::Vector3 target_pixel_coordinates;
    std_msgs::msg::String flt_mode;
    geometry_msgs::msg::Vector3 gimbal_angles;
    geometry_msgs::msg::Vector3 gimbal_state;
    //threads
    std::thread gimbal_step_thread;


    void mode_cb(const std_msgs::msg::String::SharedPtr msg){
        flt_mode = *msg;
    }
    void target_pixel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
        target_pixel_coordinates = *msg;
    }


    double atan2_2pi(double y, double x) {
        double angle = atan2(y, x); // Get angle in range [-π, π]
        if (angle < 0) {
            angle += 2 * M_PI; // Convert to [0, 2pi] range
        }
        return angle;
    }
    //inverse kinematics
    void gimbal_IK(geometry_msgs::msg::Vector3 target_pixel_coordinates, geometry_msgs::msg::Vector3 gimbal_state){
        
        //normalize from -1 to 1
        float px_n = (target_pixel_coordinates.x - IMG_CENTER_W)/IMG_CENTER_W;
        float py_n = (target_pixel_coordinates.y - IMG_CENTER_H)/IMG_CENTER_H;
        
        float px_s = sqrt(px_n*px_n + py_n*py_n);


        //calculate angle 1 delta
        float angle1_delta = atan2_2pi(py_n, px_n) - gimbal_state.x;

        //calculate angle 2 delta

        float angle2_delta = sqrt(2*CAMERA_FOV*CAMERA_FOV)*px_s;


        if (angle1_delta>0){
            gimbal_angles.x -= MAX_ROT_RATE_1;
        } else{
            gimbal_angles.x += MAX_ROT_RATE_1; 
        }

    }

    void gimbal_step_IK(geometry_msgs::Vector3 target_pixel_coordinates){



    }

    void publish_gimbal_angles(){
        while(rclcpp::ok()){
            
            
            if (flt_mode.data=="TRACKING" || flt_mode.data == "TRACK"){
            std::cout<<"TRACKING"<<std::endl;
            gimbal_IK(target_pixel_coordinates, target_pixel_coordinates);
            gimbal_angle_pub->publish(gimbal_angles);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
            }

        }
    }
    

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<GimbalControlNode>();

    rclcpp::spin(rclcppNode);

    rclcppNode->join_gimbal_step_thread();

    rclcpp::shutdown();
    return 0;
}