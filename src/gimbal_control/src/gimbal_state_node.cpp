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

class GimbalStateNode : public rclcpp::Node
{
public:

    GimbalStateNode() : Node("gimbal_state_node"){
        //std::string ns = this->get_namespace();
        std::string ns = "sjtu_drone";
       
        //subscribe to target_pixel
        std::string T_gimbal_state = "/gimbal_angles";
        std::string T_target = ns + "/target_pixel";
        std::string T_flight_mode = ns +"/mode";

        joint_state_sub = this->create_subscription

        gimbal_state_pub = this->create_publisher<geometry_msgs::msg::Vector3>(T_gimbal_state, 120);
    
        gimbal_state_thread = std::thread(&GimbalStateNode::publish_gimbal_angles, this);

    
    }

    void join_gimbal_angle_thread(){
        if (gimbal_angle_thread.joinable()){
            gimbal_angle_thread.join();
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
    geomtry_msgs::msg::Vector3 gimbal_state;
    //threads
    std::thread gimbal_state_thread;


    void publish_gimbal_angles(){
        while(rclcpp::ok()){ 
      
        std::cout<<"TRACKING"<<std::endl;
        gimbal_IK(target_pixel_coordinates, target_pixel_coordinates);
        gimbal_angle_pub->publish(gimbal_angles);
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
         

        }
    }
    

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<GimbalStateNode>();

    rclcpp::spin(rclcppNode);

    rclcppNode->join_gimbal_angle_thread();

    rclcpp::shutdown();
    return 0;
}