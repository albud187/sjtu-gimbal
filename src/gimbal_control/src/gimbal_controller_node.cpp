// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/vector3.hpp"

// class GimbalControlNode : public rclcpp::Node
// {
// public:
//     GimbalControlNode() : Node("gimbal_controller_node")
//     {
//         // Define the topic to publish
//         std::string T_gimbal_step = "/gimbal_step";

//         // Initialize the publisher
//         gimbal_step_pub = this->create_publisher<geometry_msgs::msg::Vector3>(T_gimbal_step, 40);

//         // Timer to publish the message at a fixed rate
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(25),  // 40hz interval
//             std::bind(&GimbalControlNode::publish_gimbal_step, this));

//         // Initialize the message with fixed values
//         gimbal_step.x = 0.02;
//         gimbal_step.y = 0.02;
//         gimbal_step.z = 0.0;  // Default value
//     }

// private:
//     rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_step_pub;
//     rclcpp::TimerBase::SharedPtr timer_;
//     geometry_msgs::msg::Vector3 gimbal_step;

//     void publish_gimbal_step()
//     {
//         gimbal_step_pub->publish(gimbal_step);
//     }
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<GimbalControlNode>());
//     rclcpp::shutdown();
//     return 0;
// }

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
    
    //publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_step_pub;

    //messages
    geometry_msgs::msg::Vector3 target_pixel_coordinates;
    geometry_msgs::msg::Vector3 gimbal_step;
    std_msgs::msg::String flt_mode;

    //threads
    std::thread gimbal_step_thread;


    void mode_cb(const std_msgs::msg::String::SharedPtr msg){
        flt_mode = *msg;
    }

    void target_pixel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
        target_pixel_coordinates = *msg;
    }

    geometry_msgs::msg::Vector3 gimbal_step_IK(geometry_msgs::msg::Vector3 target_pixel_coordinates){
        geometry_msgs::msg::Vector3 gimbal_step_result;
        float px_n = (target_pixel_coordinates.x - float(IMG_CENTER_W))/float(IMG_CENTER_W);
        float py_n = (target_pixel_coordinates.y - float(IMG_CENTER_H))/float(IMG_CENTER_H);
        //down and right are positive
        
        //add tiny amount to prevent division by zero issues
        float p_r = sqrt(px_n*px_n + py_n*py_n);
        float p_a = atan2(px_n, py_n);

        //big oof - p_r will always be positive

        /**
        for gimbal steps:
        x: positive = clockwise, negative = counter clockwise - rotate along same x as drone
        y: positive = look down, negative = look up - along same y axis as drone
        
        for image plane:
        px_n positive down
        py_n positive right



        **/

        //need to consider direction - the sign won't change
        gimbal_step_result.y = TEST_GIMBAL_VEL*py_n;
        //gimbal_step_result.y = TEST_GIMBAL_VEL;
        // gimbal_step_result.x = 0.01;
        // gimbal_step_result.y = 0.01;
        //angle_delta

        //goal is to zero out px_n and py_n


        std::cout<<"px_n : "<<px_n<<std::endl;
        std::cout<<"py_n : "<<py_n<<std::endl;
        std::cout<<"py_r : "<<p_r<<std::endl;
        std::cout<<"vg_y: "<<gimbal_step_result.y<<std::endl;
        
        std::cout<<" ";


        return gimbal_step_result;

    }

    void publish_gimbal_steps(){
        while(rclcpp::ok()){

            while (flt_mode.data=="TRACKING" || flt_mode.data == "TRACK"){
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