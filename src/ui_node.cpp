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
            "/turtle1/ui_cmd_vel", 10);       // create the first publisher (create_pub; specify_msg_type; topic_to_publish_to; que_size)

        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/ui_cmd_vel", 10);
    }

    // query user in a loop and fire the commander
    void run()    // returns nothing just fires the fires the commander
    {
        char chosen_turtle;     // prep a variable to hold the user input
        double linear_vel;
        double angular_vel;
        while (rclcpp::ok())
        {
            // First user input
            // Ask which turtle to move
            std::cout << "\n======================================\n";
            std::cout << "           🐢 Turtle Selector\n";
            std::cout << "======================================\n";
            std::cout << "Which turtle do you want to control?\n";
            std::cout << "  ➤ Enter '1' for Turtle 1\n";
            std::cout << "  ➤ Enter '2' for Turtle 2\n";
            std::cout << "  ➤ Enter 'q' to quit\n";
            std::cout << "--------------------------------------\n";
            std::cout << "Your choice: ";
            std::cin >> chosen_turtle;
             // checks is there is an error with the standard input stream

            if (!std::cin.good()) {       // ctrl+C also leads to here
                std::cout << "Input error. Exiting application.\n";
            break;
            }

            // If user wants to quit, quit    
            if (chosen_turtle  == 'q' || chosen_turtle  == 'Q') {
                std::cout << "Quitting application...\n";
            break;
            }

            if (chosen_turtle != '1' && chosen_turtle!= '2') {
                std::cout << "Invalid choice. Please enter a valid choise.\n";
            continue;
            }
            // Ask for linear velocity 
            std::cout << "\n======================================\n";
            std::cout << "        🚀 Linear Velocity Input\n";
            std::cout << "======================================\n";
            std::cout << "Enter linear velocity (m/s):\n";
            std::cout << "  ➤ Positive  = forward\n";
            std::cout << "  ➤ Negative  = backward\n";
            std::cout << "Examples:  1.0   -2.0    0\n";
            std::cout << "--------------------------------------\n";
            std::cout << "Linear velocity: ";
            std::cin >> linear_vel;

            if (!std::cin.good()) {
                std::cout << "Invalid linear velocity input. Exiting.\n";
                break;
            }

            // Ask for angular velocity 
            std::cout << "\n======================================\n";
            std::cout << "        🔁 Angular Velocity Input\n";
            std::cout << "======================================\n";
            std::cout << "Enter angular velocity (rad/s):\n";
            std::cout << "  ➤ Positive  = turn left\n";
            std::cout << "  ➤ Negative  = turn right\n";
            std::cout << "Examples:  1.0   -1.5    0\n";
            std::cout << "--------------------------------------\n";
            std::cout << "Angular velocity: ";
            std::cin >> angular_vel;

            std::cout << "\n✨ Command received! Executing...\n\n";

            if (!std::cin.good()) {
                std::cout << "Invalid input. Exiting application.\n";
            break;
            }

            // Logger to check the applied command  and the user choise
            RCLCPP_INFO(this->get_logger(),
                        "UI: turtle%s, linear v = %.2f, angular v=%.2f for %.1f s",
                        (chosen_turtle == '1') ? "1" : "2",
                        linear_vel,
                        angular_vel,
                        command_duration_);

            // call the function publishing 
            send_vel_cmd(chosen_turtle, linear_vel, angular_vel, command_duration_);

        }

    }
private:

    void send_vel_cmd(char chosen_turtle,
                                    double linear_vel, double angular_vel,
                                    double duration_seconds)
    {
        geometry_msgs::msg::Twist cmd_msg;

        // Set forward linear velocity, and ensure that the others are kept zero
        cmd_msg.linear.x  = linear_vel;
        cmd_msg.linear.y  = 0.0;
        cmd_msg.linear.z  = 0.0;
        cmd_msg.angular.x = 0.0;
        cmd_msg.angular.y = 0.0;
        cmd_msg.angular.z = angular_vel ;

        auto start = std::chrono::steady_clock::now();   // take the starting time stamp

        while (rclcpp::ok()) {
            auto current_time = std::chrono::steady_clock::now();   // each time in the loop, calc the elapsed time
            std::chrono::duration<double> elapsed_time = current_time- start;

            // after the duration has elapsed, leave the while loop intiating motion and stop turtle
            if (elapsed_time.count() >= duration_seconds) {
                break;
            }

            // Until the duration time elapses, publish by using the appropriate publisher depending on user choice
            if (chosen_turtle == '1') {
                pub_turtle1_->publish(cmd_msg);
            } else if (chosen_turtle == '2') {
                pub_turtle2_->publish(cmd_msg);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));     //how often should this be done ? 100 ms approximates a 10 Hz control loop
        }

        // after duration time elapses, Stop the turtle 
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;

        if (chosen_turtle == '1') {
            pub_turtle1_->publish(cmd_msg);
        } else if (chosen_turtle == '2') {
            pub_turtle2_->publish(cmd_msg);
        }

        RCLCPP_INFO(this->get_logger(),
                    "1 sec command finished. Turtle%s stopped.",
                    (chosen_turtle == '1') ? "1" : "2");

    }
        
        // Declare here in the private method
        // the two publishers 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_;
        // 
        double command_duration_; // seconds
};

// Add main
int main(int argc, char * argv[])
{
    // Initialise ROS2 with the necessary args
    rclcpp::init(argc, argv);

    // Instantiate the ui_node class
    auto node = std::make_shared<UiNode>();

    // Run the UI loop to handles user_UI
    node->run();

    // Shutdown ROS2 when we exit the loop
    rclcpp::shutdown();
    return 0;
}


