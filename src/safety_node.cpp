
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
    border_threshold(0.5);        // minimum allowed distance to boundary
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
        
        // pose callbacks - to update the pos state with the most recent one published in the topic
        // timer callback - to read that state and perform safety actions

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),   // a period of 0.1s
            std::bind(&SafetyNode::ensure_safety, this)
        );

    }
private:
    //implement the callbacks for the subscribers here
    // Pose of turtle1
    void pose1_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x1_ = msg->x;
        y1_ = msg->y;
    }

    // Pose of turtle2
    void pose2_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x2_ = msg->x;
        y2_ = msg->y;
    }

    void ensure_safety()        // Called periodically (It is like a 10 Hz control loop)
    {
        // Compute distance between turtles d = sqrt( (x1 - x2)^2 + (y1 - y2)^2 )
        double dx = x1_ - x2_;
        double dy = y1_ - y2_;
        double d  = std::sqrt(dx * dx + dy * dy);

        // Perform the safety checks
        // distance between each other
        bool too_close_to_each_other = (d < dist_threshold_);

        // how close to the safe [wall_min_, wall_max_] boundary for either x or y.

        bool t1_too_close_to_boundary =
            (std::abs(x1_ - wall_min_) < border_threshold) ||
            (std::abs(x1_ - wall_max_) < border_threshold) ||
            (std::abs(y1_ - wall_min_) < border_threshold) ||
            (std::abs(y1_ - wall_max_) < border_threshold);

        bool t2_too_close_to_boundary =
            (std::abs(x2_ - wall_min_) < border_threshold) ||
            (std::abs(x2_ - wall_max_) < border_threshold) ||
            (std::abs(y2_ - wall_min_) < border_threshold) ||
            (std::abs(y2_ - wall_max_) < border_threshold);


        // decide if stopping the turtles is necessary
        if (too_close_to_each_other || t1_too_close_to_boundary || t2_too_close_to_boundary) {
            // Build zero velocity STOP command
            geometry_msgs::msg::Twist stop_msg;
            stop_msg.linear.x  = 0.0;
            stop_msg.linear.y  = 0.0;
            stop_msg.linear.z  = 0.0;
            stop_msg.angular.x = 0.0;
            stop_msg.angular.y = 0.0;
            stop_msg.angular.z = 0.0;

            // Send stop command to both turtles. It is like freezing everything if unsafe
            // ** if time available, implement smart stopping than just stopping both
            pub_t1_cmd_->publish(stop_msg);
            pub_t2_cmd_->publish(stop_msg);
        }
    }

    // define the member variables 
    
    // Subscribers: to the turtles' positions
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t1_pose_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t2_pose_;

    // Publishers: overriding velocity commands to stop the robots
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t1_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t2_cmd_;

    // Timer for periodic safety checks (how often should dist be calc)
    rclcpp::TimerBase::SharedPtr timer_;

    // Stored pose information
    double x1_, y1_;
    double x2_, y2_;

    // Safety parameters 
    double dist_threshold_;   // minimum allowed distance between turtles
    double border_threshold;  // minimum allowed distance to boundary
    double wall_min_;         // safe zone
    double wall_max_;         





};