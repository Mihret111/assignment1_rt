
#include <cmath>                          // needed for calcualting distance
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"         // msg to retrieve pose of each from
#include "geometry_msgs/msg/twist.hpp"    // msg to stop each incase of dangerous condition
#include "std_msgs/msg/float32.hpp"       // msg to send the distance value   (used just for logging purpose)


using std::placeholders::_1;              // Needed for std::bind to connect callbacks to subscribers/timers

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode()
    : rclcpp::Node("safety_node"),  // Node name: safety node
    have_cmd1_(false),              // true, if user cmd(from UI_node) becomes available
    have_cmd2_(false),

    // TODO: make these params user-configurable later
    dist_threshold_(1.5),           // minimum allowed distance between turtles
    border_threshold(0.1),          // minimum allowed distance to boundary

    // definition of the safe exploration area
    wall_min_(1.0),                 // inner boundary in turtlesim world
    wall_max_(10.0)                 // outer boundary in turtlesim world
    {
        // Pose subscribers: listen and store the turtles' positions
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

        // UI command subscribers: listen and store user-desired commands from UiNode
        sub_t1_ui_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/ui_cmd_vel", 10,
            std::bind(&SafetyNode::ui_cmd1_callback, this, _1)
        );

        sub_t2_ui_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle2/ui_cmd_vel", 10,
            std::bind(&SafetyNode::ui_cmd2_callback, this, _1)
        );

        // distance publisher: publish the distance between turtles to be used for monitoring 
        pub_distance_ = this->create_publisher<std_msgs::msg::Float32>(
            "/turtles_distance",
            10
        );

        // cmd_vel publishers: publish the cmd_vel to turtlesim ( the only visible one to turtlesim)
        pub_t1_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);

        pub_t2_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/cmd_vel", 10);

        // Timer callback: periodic control loop (20 Hz) to check safety and publish commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),   // a period of 0.02s, control loop: 20 Hz
            std::bind(&SafetyNode::ensure_safety, this)
        );

    }
private:
    // Pose of turtle1
    void pose1_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x1_ = msg->x;
        y1_ = msg->y;
        //retrieve the heading angle theta: associate in safety decisions
        theta1_ = msg->theta;
    }

    // Pose of turtle2
    void pose2_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x2_ = msg->x;
        y2_ = msg->y;
        //retrieve the heading angle theta: associate in safety decisions
        theta2_ = msg-> theta;
    }

    // UI command callbacks 
    void ui_cmd1_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_t1_ = *msg;
        have_cmd1_ = true;
    }

    void ui_cmd2_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_t2_ = *msg;
        have_cmd2_ = true;
    }

    void ensure_safety()        // Called periodically (equivalent to a 20 Hz control loop)
    {
        if (!have_cmd1_ && !have_cmd2_) {   
            // if both cmd is not available, then
            // nothing to send to the tutles
            return;
        }
        // Compute distance between turtles
        double dx = x1_ - x2_;
        double dy = y1_ - y2_;
        double d  = std::sqrt(dx * dx + dy * dy);

        // distance msg for monitoring
        std_msgs::msg::Float32 d_msg;
        d_msg.data = static_cast<float>(d);
        pub_distance_->publish(d_msg);     // publish

        // Start with assuming UI_node cmd as safe (subjected to test later)
        geometry_msgs::msg::Twist safe_cmd1 = last_cmd_t1_;
        geometry_msgs::msg::Twist safe_cmd2 = last_cmd_t2_;

        // A lambda expression to set twist to zero (to be used for unsafe encounters)
        auto make_zero = [](geometry_msgs::msg::Twist & msg){
            msg.linear.x  = 0.0;
            msg.linear.y  = 0.0;
            msg.linear.z  = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;
        };

        // Perform the safety checks , 
            // Update safe_cmd1/2 if necessary and log reason for stopping

        // Safety separation check
        bool too_close_to_each_other = (d < dist_threshold_);
        if (too_close_to_each_other) {
            // Approximate world velocities from linear.x along heading
            double v1x = last_cmd_t1_.linear.x * std::cos(theta1_);
            double v1y = last_cmd_t1_.linear.x * std::sin(theta1_);
            double v2x = last_cmd_t2_.linear.x * std::cos(theta2_);
            double v2y = last_cmd_t2_.linear.x * std::sin(theta2_);

            // No commanded motion -> no logs
            double speed_norm = std::fabs(last_cmd_t1_.linear.x) + std::fabs(last_cmd_t2_.linear.x);
            const double EPS = 1e-3;
            if (speed_norm < EPS) {
                // No active motion command: just stay as we are, .
                // (safe_cmd1/2 inherit last_cmd or zero(if stopped))   
            } else {
                // Relative position & velocity
                double rv_dot = dx * (v1x - v2x) + dy * (v1y - v2y);

                // rv_dot < 0  -> moving closer
                // rv_dot > 0  -> moving apart
                if (rv_dot < 0.0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                        "Turtles too close (d=%.2f < %.2f). Stopping both.",
                                        d, dist_threshold_);
   
                    make_zero(safe_cmd1);
                    make_zero(safe_cmd2); 
                }
                else{
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                        "Escaping command allowed.");
                }
            }
    } 
        // Safety against boundary check
            // how close to the safe [wall_min_, wall_max_] boundary for either x or y.
        
        //Turtlle-1
        double v1x = last_cmd_t1_.linear.x * std::cos(theta1_);     // Approx approximate world velocity of turtle1
        double v1y = last_cmd_t1_.linear.x * std::sin(theta1_);
    
        bool t1_too_close_to_boundary = (std::min({std::abs(x1_ - wall_min_), std::abs(x1_ - wall_max_), std::abs(y1_ - wall_min_), std::abs(y1_ - wall_max_)}) < border_threshold);

        if (t1_too_close_to_boundary) {
            double speed1 = std::fabs(last_cmd_t1_.linear.x);       // strength of the motion command
            const double EPS = 1e-3;
            // check if cmd is forcing to push out more to the boundary or not (worsening situation or the opposite)
            bool pushing_outward =
                ((std::abs(x1_ - wall_min_) < border_threshold)&& v1x < 0.0) ||   // already left, moving further left
                ((std::abs(x1_ - wall_max_) < border_threshold) && v1x > 0.0) ||   // already right, moving further right
                ((std::abs(y1_ - wall_min_) < border_threshold) && v1y < 0.0) ||   // already bottom, moving further down
                ((std::abs(y1_ - wall_max_) < border_threshold) && v1y > 0.0);     // already top, moving further up

            if (pushing_outward && speed1 > EPS){

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Turtle1 near boundary: x=%.2f, y=%.2f. Cannot move further.",
                x1_, y1_);

                make_zero(safe_cmd1);
            }
            else if (speed1 > EPS) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                     2000,
                                     "Escaping command allowed for turtle1.");
            }
            // If speed1 <= EPS (no effective command has been perceived), so stay idle.
        }

        //Turtlle-2
        double v2x = last_cmd_t2_.linear.x * std::cos(theta2_);
        double v2y = last_cmd_t2_.linear.x * std::sin(theta2_);
        
        bool t2_too_close_to_boundary = (std::min({std::abs(x2_ - wall_min_), std::abs(x2_ - wall_max_), std::abs(y2_ - wall_min_), std::abs(y2_ - wall_max_)}) < border_threshold);
        if (t2_too_close_to_boundary) {
            double speed2 = std::fabs(last_cmd_t2_.linear.x);    // retrieve the linear cmd
            const double EPS = 1e-3;
            // check if cmd is forcing to push out more to the boundary
            bool pushing_outward =
                ((std::abs(x2_ - wall_min_) < border_threshold) && v2x < 0.0) ||   // already left, moving further left
                ((std::abs(x2_ - wall_max_) < border_threshold) && v2x > 0.0) ||   // already right, moving further right
                ((std::abs(y2_ - wall_min_) < border_threshold) && v2y < 0.0) ||   // already bottom, moving further down
                ((std::abs(y2_ - wall_max_) < border_threshold) && v2y > 0.0);     // already top, moving further up
            if (pushing_outward && speed2 > EPS){
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Turtle2 near boundary: x=%.2f, y=%.2f. Cannot move further.",
                            x2_, y2_);
                make_zero(safe_cmd2);}
            else if (speed2 > EPS)  {
                // Only log if there is a meaningful inward command
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                    2000,
                                    "Escaping command allowed for turtle2.");
                }
        }
            // Send stop command to both turtles. Freezing if unsafe
            // Publish final safe commands to turtles
            if (have_cmd1_) {
                pub_t1_cmd_->publish(safe_cmd1);
            }
            if (have_cmd2_) {
                pub_t2_cmd_->publish(safe_cmd2);
            }
        
    }

    // define the member variables 
    
    // Subscribers: to the turtles' positions
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t1_pose_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t2_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_t1_ui_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_t2_ui_cmd_;


    // Publishers: overriding velocity commands to stop the robots
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distance_;    // distance information for loggin
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t1_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t2_cmd_;

    // Timer for periodic safety checks (how often safety distances are calculated)
    rclcpp::TimerBase::SharedPtr timer_;

    // Stored pose information
    double x1_, y1_, theta1_;
    double x2_, y2_, theta2_;

    // Last desired commands from UI
    geometry_msgs::msg::Twist last_cmd_t1_;
    geometry_msgs::msg::Twist last_cmd_t2_;
    bool have_cmd1_;
    bool have_cmd2_;

    // Safety parameters 
    double dist_threshold_;   // minimum allowed distance between turtles
    double border_threshold;  // minimum allowed distance to boundary
    double wall_min_;         // safe zone min
    double wall_max_;         // safe zone max

};

// Instantiate safety_node and let the callback functions run
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SafetyNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}