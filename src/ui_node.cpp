#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"              // rclcpp: ros2 client library which includes all the commands
#include "geometry_msgs/msg/twist.hpp"    // msg to send as a candidate cmd_vel to turtles

#include "util.h"

class UiNode : public rclcpp::Node        // icreating a node
{   
    // needs 2 publishers and no subscribed info is demanded.
    
#include "turtlesim/srv/spawn.hpp"        // msg to spawn a new turtle
public:    
    UiNode()                      // define the constructor                   
    : rclcpp::Node("ui_node"),    // define node name
    command_duration_(1.0)        // For how long will the command be applied
    {
        // candidate cmd_vel publishers: publish the cmd_vel to topic ui_cmd_vel
        pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/ui_cmd_vel", 10);       // 

        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/ui_cmd_vel", 10);

        pub_turtle3_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle3/ui_cmd_vel", 10);
    }

    // UI : to query user input in a loop and fire the commander
    void run()    
    {
        std::string chosen_turtle;     // prepare variable to hold the user input

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
            std::cout << "  ➤ Enter '3' for Turtle 3\n";
            std::cout << "  ➤ Enter 'q' to quit\n";
            std::cout << "--------------------------------------\n";
            std::cout << "Your choice: ";
            std::cin >> chosen_turtle;

            // checks is there is an error with the standard input stream
            if (!std::cin.good()) {       // ctrl+C also leads to here
                std::cout << "\n\t\tInput error. Exiting application.\n";
            break;
            }

            // handle user preference to quit the program    
            if (chosen_turtle  == "q" || chosen_turtle  == "Q") {
                std::cout << "\n\t\tQuitting application...\n";
            break;
            }

            // handle user preference to select a turtle
            if (chosen_turtle != "1" && chosen_turtle!= "2" && chosen_turtle!= "3") {
                std::cout << "\n\t\tInvalid choice. Please enter a valid choise.\n";
            continue;
            }

            // ask for linear velocity 
            double linear_vel = InquirelinearVelocity();

            // ask for angular velocity 
            double angular_vel = InquireangularVelocity();


            std::cout << "\n\t\t✨ Command received! \n\n";
            // logger to check the applied command  and the user choise
            RCLCPP_INFO(this->get_logger(),
                        "\t\tUI Node: turtle%s, linear v = %.2f, angular v=%.2f for %.1f s",
                        (chosen_turtle == "1") ? "1" : "2" : "3",
                        linear_vel,
                        angular_vel,
                        command_duration_);

            // send to function publishing the velocity command to the topic ui_cmd_vel
            send_vel_cmd(chosen_turtle, linear_vel, angular_vel, command_duration_);

        }

    }
private:

    double InquirelinearVelocity()
    {
        while (rclcpp::ok()) {
            std::string linear_vel_str;
            // ask for linear velocity 
            std::cout << "\n======================================\n";
            std::cout << "        🚀 Linear Velocity Input\n";
            std::cout << "======================================\n";
            std::cout << "Enter linear velocity (m/s):\n";
            std::cout << "  ➤ Positive  = forward\n";
            std::cout << "  ➤ Negative  = backward\n";
            std::cout << "Examples:  1.0   -2.0    0\n";
            std::cout << "--------------------------------------\n";
            std::cout << "Linear velocity: ";
            std::cin >> linear_vel_str;

            if (!isNumber(linear_vel_str)) {
                std::cout << "\n\t\tInvalid linear velocity. Please enter a valid value.\n";
            continue;
            }
            double linear_vel = std::stod(linear_vel_str);
            if (!std::cin.good()) {
                std::cout << "\n\t\tInput error. Exiting application.\n";
                break;
            }
            return linear_vel;
        }
        return 0;
    }
    double InquireangularVelocity()
    {
        while (rclcpp::ok()) {
            std::string angular_vel_str;
            // ask for angular velocity 
            std::cout << "\n======================================\n";
            std::cout << "        🔁 Angular Velocity Input\n";
            std::cout << "======================================\n";
            std::cout << "Enter angular velocity (rad/s):\n";
            std::cout << "  ➤ Positive  = turn left\n";
            std::cout << "  ➤ Negative  = turn right\n";
            std::cout << "Examples:  1.0   -1.5    0\n";
            std::cout << "--------------------------------------\n";
            std::cout << "Angular velocity: ";
            std::cin >> angular_vel_str;


            if (!isNumber(angular_vel_str)) {
                std::cout << "\n\t\tInvalid angular velocity. Please enter a valid value.\n";
            continue;
            }
            if (!std::cin.good()) {
                std::cout << "\n\t\tInput error. Exiting application.\n";
            break;
            }
            double angular_vel = std::stod(angular_vel_str);
            return angular_vel;
        }
        return 0;
    }

    void send_vel_cmd(std::string chosen_turtle,
                                    double linear_vel, double angular_vel,
                                    double duration_seconds)
    {
        geometry_msgs::msg::Twist cmd_msg;

        // Set forward horizontal linear velocity and normal angular velocity
        // Ensure others are kept zero
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
            if (chosen_turtle == "1") {
                pub_turtle1_->publish(cmd_msg);
            } else if (chosen_turtle == "2") {
                pub_turtle2_->publish(cmd_msg);
            } else if (chosen_turtle == "3") {
                pub_turtle3_->publish(cmd_msg);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));     // how often should this be done ? 100 ms approximates a 10 Hz control loop
        }

        // after duration_seconds elapse, send stop command 
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;

        if (chosen_turtle == "1") {
            pub_turtle1_->publish(cmd_msg);
        } else if (chosen_turtle == "2") {
            pub_turtle2_->publish(cmd_msg);
        } else if (chosen_turtle == "3") {
            pub_turtle3_->publish(cmd_msg);
        }

        RCLCPP_INFO(this->get_logger(),
                    "1 sec command finished. Turtle%s stopped.",
                    (chosen_turtle == "1") ? "1" : "2" : "3");

    }
        
     // define the member variables 

    // Publishers: candidate velocity commands 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle3_;
        double command_duration_; // apply cmd for this duration
};

// Instantiate ui_node 
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


