
#include <cmath>                       // needed for calcualting distance
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"      // msg to retrieve pose of each from
#include "geometry_msgs/msg/twist.hpp"    // msg type to stop each incase of dangerous condition
#include "std_msgs/msg/float32.hpp"       // to send the distance value   (if needed later)

// Needed for std::bind to connect callbacks to subscribers/timers
using std::placeholders::_1;

class SafetyNode : public rclcpp::Node
{
public:
    SafetyNode()
    : rclcpp::Node("safety_node"),  // Node name inside the ROS graph
    have_cmd1_(false),              // true, if user cmd(from UI_node) becomes available
    have_cmd2_(false),
    // default val for the vars
    dist_threshold_(1.5),          // minimum allowed distance between turtles
    border_threshold(0.1),        // minimum allowed distance to boundary
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

        // UI command subscribers  (user-desired commands from UiNode)
        sub_t1_ui_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/ui_cmd_vel", 10,
            std::bind(&SafetyNode::ui_cmd1_callback, this, _1)
        );

        sub_t2_ui_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle2/ui_cmd_vel", 10,
            std::bind(&SafetyNode::ui_cmd2_callback, this, _1)
        );

        // distance b/n turtles publisher : to be used for monitoring 
        pub_distance_ = this->create_publisher<std_msgs::msg::Float32>(
            "/turtles_distance",
            10
        );

        // Cmd_vel publishers (the only one turtlesim sees)
        pub_t1_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10);

        pub_t2_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/cmd_vel", 10);

        // callbacks needed
        // pose callbacks - to update the pos state with the most recent one published in the topic
        // timer callback - to read that state and perform safety actions

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),   // a period of 0.02s, control loop: 20 Hz
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
        //retrieve the heading angle theta: later used to deside safety along with rel velocity
        theta1_ = msg->theta;
    }

    // Pose of turtle2
    void pose2_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x2_ = msg->x;
        y2_ = msg->y;
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

    void ensure_safety()        // Called periodically (It is like a 10 Hz control loop)
    {
        // Need at least some commands
        // This assumes an intially safe position of the turtles
        //TODO: So need to ensure that the spawner doesn't violate this

        if (!have_cmd1_ && !have_cmd2_) {   
            // if both cmd is not available, then
            // nothing to send to the tutles
            return;
        }
        // Compute distance between turtles d = sqrt( (x1 - x2)^2 + (y1 - y2)^2 )
        double dx = x1_ - x2_;
        double dy = y1_ - y2_;
        double d  = std::sqrt(dx * dx + dy * dy);

        // distance msg for monitoring
        std_msgs::msg::Float32 d_msg;
        d_msg.data = static_cast<float>(d);
        pub_distance_->publish(d_msg);     // publish

        // First assign the desired commands from UI_node as safe cmd(subjected to test later)
        geometry_msgs::msg::Twist safe_cmd1 = last_cmd_t1_;
        geometry_msgs::msg::Twist safe_cmd2 = last_cmd_t2_;

        // A lambda expression to set twist to zero :to be used for unsafe encounters
        auto make_zero = [](geometry_msgs::msg::Twist & msg){
            msg.linear.x  = 0.0;
            msg.linear.y  = 0.0;
            msg.linear.z  = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;
        };

        // decide if stopping the turtles is necessary
            // Log WHY we are stopping (for debugging)

        // Perform the safety checks
        // distance between each other
        bool too_close_to_each_other = (d < dist_threshold_);
        if (too_close_to_each_other) {
            // Approximate world velocities from linear.x along heading
            double v1x = last_cmd_t1_.linear.x * std::cos(theta1_);
            double v1y = last_cmd_t1_.linear.x * std::sin(theta1_);
            double v2x = last_cmd_t2_.linear.x * std::cos(theta2_);
            double v2y = last_cmd_t2_.linear.x * std::sin(theta2_);

            // If there is essentially no commanded motion, don't put logs
            double speed_norm = std::fabs(last_cmd_t1_.linear.x) + std::fabs(last_cmd_t2_.linear.x);
            const double EPS = 1e-3;
            if (speed_norm < EPS) {
                // No active motion command: just stay as we are, no "allowing escape" noise.
                // (safe_cmd1/2 already inherit zero or last_cmd)   
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
                                        "Turtles too close (d=%.2f), but commands move them apart. Allowing escape.",
                                        d);
                }
            }
    } 
        // Safety against boundary  
        // how close to the safe [wall_min_, wall_max_] boundary for either x or y.
        
        //Turtlle-1
        // Components of the commanded vel in x and y
        // Approx approximate world velocity of turtle1
        double v1x = last_cmd_t1_.linear.x * std::cos(theta1_);
        double v1y = last_cmd_t1_.linear.x * std::sin(theta1_);
    
        double dx_min = std::abs(x1_ - wall_min_);
        double dx_max = std::abs(x1_ - wall_max_);
        double dy_min = std::abs(y1_ - wall_min_);
        double dy_max = std::abs(y1_ - wall_max_);
        
        bool t1_too_close_to_boundary = (std::min({dx_min, dx_max, dy_min, dy_max}) < border_threshold);

        if (t1_too_close_to_boundary) {
            // how "strong" is the motion command?
            double speed1 = std::fabs(last_cmd_t1_.linear.x);
            const double EPS = 1e-3;
            // check if cmd is forcing to push out more to the boundary
            bool pushing_outward =
                ((dx_min < border_threshold)&& v1x < 0.0) ||   // already left, moving further left
                ((dx_max < border_threshold) && v1x > 0.0) ||   // already right, moving further right
                ((dy_min < border_threshold) && v1y < 0.0) ||   // already bottom, moving further down
                ((dy_max < border_threshold) && v1y > 0.0);     // already top, moving further up

            if (pushing_outward && speed1 > EPS){

                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Turtle1 near boundary: x=%.2f, y=%.2f. Cannot move further.",
                x1_, y1_);

                make_zero(safe_cmd1);
            }
            else if (speed1 > EPS) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                     2000,
                                     "Turtle1 near boundary (x=%.2f, y=%.2f) but command "
                                     "points inward. Allowing motion.",
                                     x1_, y1_);
            }
            // If speed1 <= EPS (no effective command), stay quiet.
        }

            //Turtlle-2
        // Components of the commanded vel in x and y
        // Approx approximate world velocity of turtle1
        double v2x = last_cmd_t2_.linear.x * std::cos(theta2_);
        double v2y = last_cmd_t2_.linear.x * std::sin(theta2_);

        dx_min = std::abs(x2_ - wall_min_);
        dx_max = std::abs(x2_ - wall_max_);
        dy_min = std::abs(y2_ - wall_min_);
        dy_max = std::abs(y2_ - wall_max_);
        
        bool t2_too_close_to_boundary = (std::min({dx_min, dx_max, dy_min, dy_max}) < border_threshold);

/*         bool t2_too_close_to_boundary =
            (std::abs(x2_ - wall_min_) < border_threshold) ||
            (std::abs(x2_ - wall_max_) < border_threshold) ||
            (std::abs(y2_ - wall_min_) < border_threshold) ||
            (std::abs(y2_ - wall_max_) < border_threshold); */

        if (t2_too_close_to_boundary) {
            double speed1 = std::fabs(last_cmd_t1_.linear.x);    // retrieve the linear cmd
            // TODO check from sim (if the angular would be advantageous)
            const double EPS = 1e-3;
            // check if cmd is forcing to push out more to the boundary
            bool pushing_outward =
                ((dx_min < border_threshold) && v2x < 0.0) ||   // already left, moving further left
                ((dx_max < border_threshold) && v2x > 0.0) ||   // already right, moving further right
                ((dy_min < border_threshold) && v2y < 0.0) ||   // already bottom, moving further down
                ((dy_max < border_threshold) && v2y > 0.0);     // already top, moving further up
            if (pushing_outward){
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Turtle2 near boundary: x=%.2f, y=%.2f. Cannot move further.",
                            x2_, y2_);
                make_zero(safe_cmd2);}
            else if (speed1 > EPS)  {
                // Only log if there is a meaningful inward command
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                    2000,
                                    "Turtle2 near boundary (x=%.2f, y=%.2f) but command "
                                    "points inward. Allowing motion.",
                                    x2_, y2_);
                }
        }
            // Send stop command to both turtles. It is like freezing everything if unsafe
            // ** if time available, implement smart stopping than just stopping both

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
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distance_;    // distance information
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t1_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_t2_cmd_;

    // Timer for periodic safety checks (how often should dist be calc)
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
    double wall_min_;         // safe zone
    double wall_max_;         

};

// Add main to initializes ROS, instantiate safety_node and then let the callback funcs run
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SafetyNode>();

    // rclcpp::spin() blocks and processes callbacks (ie: the subscribers and the timers)
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}