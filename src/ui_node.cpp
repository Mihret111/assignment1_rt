#include <iostream>

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
    void run()    // returns nth just fires the fires the commander
    {
        char chosen_turtle// prep a variable to hold the user input
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

            if (turtle_choice != '1' && turtle_choice != '2') {
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

        }
    }




}


