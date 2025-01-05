#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class GimbalControlNode : public rclcpp::Node
{
public:
    GimbalControlNode() : Node("gimbal_controller_node")
    {
        // Define the topic to publish
        std::string T_gimbal_step = "/gimbal_step";

        // Initialize the publisher
        gimbal_step_pub = this->create_publisher<geometry_msgs::msg::Vector3>(T_gimbal_step, 40);

        // Timer to publish the message at a fixed rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25),  // 40hz interval
            std::bind(&GimbalControlNode::publish_gimbal_step, this));

        // Initialize the message with fixed values
        gimbal_step.x = 0.02;
        gimbal_step.y = 0.02;
        gimbal_step.z = 0.0;  // Default value
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_step_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Vector3 gimbal_step;

    void publish_gimbal_step()
    {
        gimbal_step_pub->publish(gimbal_step);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalControlNode>());
    rclcpp::shutdown();
    return 0;
}
