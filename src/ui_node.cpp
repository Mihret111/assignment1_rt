#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"              // rclcpp is the ros2 client library which includes all the commands
#include "geometry_msgs/msg/twist.hpp"    // this is the msg type we aim to prep (cmd_vel)

class UiNode : public rclcpp::Node     // inheriting from Node makes it a node
{          // just need to have 2 publishers and no subscribed info is needed
public:    
    // def the constructor
    UiNode()                      // Const<- is public because we need to access it from outside                         
    : rclcpp::Node("ui_node"),    // name of the node = ui_node
    command_duration_(1.0)        // For how long
    {
        pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);       // the first publisher (create_pub; specify_msg_type; topic_to_publish_to; que_size)

        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/cmd_vel", 10);
    }

    // query user in a loop and fire the commander
    void run()    // returns nothing just fires the fires the commander
    {
        char chosen_turtle     // prep a variable to hold the user input
        double velocity
        while (rclcpp::ok())
        {
            // First user input
            std::cout<< "Which turtle do you want ot move?\n"
            << "Enter 1 or 2 ";      // ** later distinguish from the two after spawning and visualizing
            std::cin>> chosen_turtle;    // checks is there is an error with the standard input stream

            if (!std::cin.good()) {
                std::cout << "Input error. Exiting application.\n";
            break;
            }

            if (chosen_turtle != '1' && chosen_turtle!= '2') {
                std::cout << "Invalid choice. Please, enter 1 or 2.\n";
            continue;
            }

            //Second user input
            std::cout << "Enter the desired linear velocity: ";
            std::cin >> velocity;

            if (!std::cin.good()) {
                std::cout << "Invalid input. Exiting application.\n";
            break;
            }

            // call the function publishing 
            send_vel_cmd(chosen_turtle, velocity,duration)

        }

    }

    void send_velocity_for_duration(char turtle_choice,
                                    double velocity,
                                    double duration_seconds)
    {
        geometry_msgs::msg::Twist cmd_msg;

        // Set forward linear velocity, and ensure that the others are kept zero
        cmd_msg.linear.x  = velocity;
        cmd_msg.linear.y  = 0.0;
        cmd_msg.linear.z  = 0.0;
        cmd_msg.angular.x = 0.0;
        cmd_msg.angular.y = 0.0;
        cmd_msg.angular.z = 0.0;

        auto start = std::chrono::steady_clock::now();   // take the starting time stamp

        while (rclcpp::ok()) {
            auto current_time = std::chrono::steady_clock::now();   // each time in the loop, calc the elapsed time
            std::chrono::duration<double> elapsed_time = current_time- start;

            // after the duration has elapsed, leave the while loop intiating motion and stop turtle
            if (elapsed_time.count() >= duration_seconds) {
                break;
            }

            // Until the duration time elapses, publish by using the appropriate publisher depending on user choice
            if (turtle_choice == '1') {
                pub_turtle1_->publish(cmd_msg);
            } else if (turtle_choice == '2') {
                pub_turtle2_->publish(cmd_msg);
            }

            std::this_thread::sleep_for(100ms);      //how often should this be done ? 100 ms approximates a 10 Hz control loop
        }

        // after duration time elapses, Stop the turtle 
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;

        if (turtle_choice == '1') {
            pub_turtle1_->publish(cmd_msg);
        } else if (turtle_choice == '2') {
            pub_turtle2_->publish(cmd_msg);
        }
    }





}


