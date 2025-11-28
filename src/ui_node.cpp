#include <iostream>

#include "rclcpp/rclcpp.hpp"              // rclcpp is the ros2 client library which includes all the commands
#include "geometry_msgs/msg/twist.hpp"    // this is the msg type we aim to prep (cmd_vel)

class UiNode : public rclcpp::Node     // inheriting from Node makes it a node
{          // just need to have 2 publishers and no subscribed info is needed
public:    
    UiNode()                      // Const<- is public because we need to access it from outside                         
    : rclcpp::Node("ui_node"),    // name of the node = ui_node
    command_duration_(1.0)        // For how long
    {
        pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);       // the first publisher (create_pub; specify_msg_type; topic_to_publish_to; que_size)

        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/cmd_vel", 10);
    }

}
