#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
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
        
        flight_mode_sub = this->create_subscription<std_msgs::msg::String>(
            T_flight_mode, 10, std::bind(&GimbalControlNode::mode_cb, this, std::placeholders::_1));
        
        target_pixel_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
            T_target, 50, std::bind(&GimbalControlNode::target_pixel_cb, this, std::placeholders::_1));

        gimbal_angle_pub = this->create_publisher<geometry_msgs::msg::Vector3<T_gimbal_angles, 60);
    
        gimbal_angle_thread = std::thread(&GimbalControlNode::publish_gimbal_angles, this);

    
    }

    void join_gimbal_angle_thread(){
        if (gimbal_angle_thread.joinable()){
            gimbal_angle_thread.join();
        }
    }

private:
    std::mutex mutex_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_mode_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_pixel_sub;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_angle_pub;

    geometry_msgs::msg::Vector3 target_pixel_coordinates;
    std_msgs::msg::String flt_mode;

    void mode_cb(const std_msgs::msg::String::SharedPtr msg){
        flt_mode = *msg;
    }
    void target_pixel_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
        target_pixel_coordinates = *msg;
    }

    //inverse kinematics
    geometry_msgs::msg::Vector3 gimbal_IK(geometry_msgs::msg::Vector3 target_pixel_coordinates){
        geometry_msgs::msg::Vector3 gimbal_angles;
        int d_px = target_pixel_coordinates.x - IMG_CENTER_W;
        int d_py = target_pixel_coordinates.y - IMG_CENTER_H;

        //use pixel angle to calculate angle1
        float angle1 = atan2(d_py, d_px);

        //use pixel distance from optical axis to claculate angle2
        float optical_axis_pixel_distance = sqrt(d_px**2 + d_py**2)

        float angle2 = atan2(optical_axis_pixel_distance, FOCAL_LEN);

        gimbal_angles.x = angle1;
        gimbal_angles.y = angle2;

        return gimbal_angles;
    }

    void publish_gimbal_angle(){
        while(rclcpp::ok()){
            geometry_msgs::msg::Vector3 gimbal_angles = gimbal_IK(target_pixel_coordinates);
            gimbal_angle_pub->publish(gimbal_angles);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
    

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto rclcppNode = std::make_shared<GimbalControlNode>()

    rclcpp::spin(rclcppNode);

    rclcppNode->join_gimbal_angle_thread();

    rclcpp::shutdown();
    return 0;
}