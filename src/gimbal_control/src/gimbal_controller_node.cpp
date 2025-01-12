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

    GimbalControlNode() : Node("gimbal_controller_node"){
        //std::string ns = this->get_namespace();
        std::string ns = "sjtu_drone";
       
        //subscribe to target_pixel
        std::string T_target = ns + "/target_pixel";
        std::string T_flight_mode = ns +"/mode";
        std::string T_gimbal_step = "/gimbal_step";
        std::string T_gimbal_state = "/gimbal_state";

        flight_mode_sub = this->create_subscription<std_msgs::msg::String>(
            T_flight_mode, 10, std::bind(&GimbalControlNode::mode_cb, this, std::placeholders::_1));
        
        target_pixel_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            T_target, 50, std::bind(&GimbalControlNode::target_pixel_cb, this, std::placeholders::_1));

        gimbal_state_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            T_gimbal_state, 50, std::bind(&GimbalControlNode::gimbal_state_cb, this, std::placeholders::_1));

        gimbal_step_pub = this->create_publisher<geometry_msgs::msg::Vector3>(T_gimbal_step, 20);
    
        gimbal_step_thread = std::thread(&GimbalControlNode::publish_gimbal_steps, this);

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
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_state_sub;
    //publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_step_pub;
    
    //messages
    geometry_msgs::msg::Vector3 target_pixel_coordinates;
    geometry_msgs::msg::Vector3 gimbal_step;
    std_msgs::msg::String flt_mode;
    geometry_msgs::msg::Vector3 previous_gimbal_step;
    geometry_msgs::msg::Vector3 gimbal_state;
    //threads
    std::thread gimbal_step_thread;


    void mode_cb(const std_msgs::msg::String::SharedPtr msg){
        flt_mode = *msg;
    }

    void target_pixel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
        target_pixel_coordinates = *msg;
    }
    void gimbal_state_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
        gimbal_state = *msg;
    }


    geometry_msgs::msg::Vector3 gimbal_step_IK(geometry_msgs::msg::Vector3 target_pixel_coordinates) {
        geometry_msgs::msg::Vector3 gimbal_step_result;

        // Normalize pixel coordinates to [-1, 1]
        float px_n = (target_pixel_coordinates.x - IMG_CENTER_W) / IMG_CENTER_W;
        float py_n = (target_pixel_coordinates.y - IMG_CENTER_H) / IMG_CENTER_H;

        // Compute distance (radius) from the image center
        float p_r = std::sqrt(px_n * px_n + py_n * py_n);
        float angle_delta = atan2(py_n, px_n)-PI/2;
        float angle_delta_deg = angle_delta*180/PI;

        gimbal_step_result.x = -TEST_GIMBAL_VEL*px_n;
        gimbal_step_result.y = TEST_GIMBAL_VEL*py_n;
        
        // Debugging output for monitoring behavior
        std::cout << "px_n : " << px_n << std::endl;
        std::cout << "py_n : " << py_n << std::endl;
        std::cout << "p_r : " << p_r << std::endl;
        std::cout << "angle_delta : "<<angle_delta_deg<<std::endl;
        // std::cout << "gimbal_step.x (joint 1): " << gimbal_step_result.x << std::endl;
        // std::cout << "gimbal_step.y (joint 2): " << gimbal_step_result.y << std::endl;
        std::cout << " " << std::endl;

        return gimbal_step_result;
    }
        void publish_gimbal_steps(){
        while(rclcpp::ok()){

            if (flt_mode.data=="TRACKING" || flt_mode.data == "TRACK"){
            std::cout<<"TRACKING"<<std::endl;
            gimbal_step = gimbal_step_IK(target_pixel_coordinates);
            gimbal_step_pub->publish(gimbal_step);

            //publish at 50 hz, or 20 ms
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
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