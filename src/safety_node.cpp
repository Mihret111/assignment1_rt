
#include <cmath>                       // needed for calcualting distance
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"      // msg to retrieve pose of each from
#include "geometry_msgs/msg/twist.hpp"    // msg type to stop each incase of dangerous condition
#include "std_msgs/msg/float32.hpp"       // to send the distance value   

// Needed for std::bind to connect callbacks to subscribers/timers
using std::placeholders::_1;

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode()
    : rclcpp::Node("safety_node"),  // Node name inside the ROS graph
    // default val for the vars
    dist_threshold_(1.0),          // minimum allowed distance between turtles

    // def oundary of the the safe exploration area
    wall_min_(1.0),                // inner boundary in turtlesim world
    wall_max_(10.0)                // outer boundary in turtlesim world
    {
        // Pose subscribers: listen and store their positions
        sub_t1_pose_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",          // topic name
            10,                       // queue size
            std::bind(&SafetyNode::pose1_callback, this, _1)
        );

        sub_t2_pose_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose",
            10,
            std::bind(&SafetyNode::pose2_callback, this, _1)
        );

        // Cmd_vel publishers (to override normal vel while in danger)
        pub_t1_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);

        pub_t2_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/cmd_vel", 10);

        // callbacks needed
        
        // pose callbacks - to update the pos state with the most recent 
        // one published in the topic
        // timer callback - to read that state and decide safety actions

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),   // a period of 0.1s
            std::bind(&SafetyNode::timer_callback, this)
        );

    }