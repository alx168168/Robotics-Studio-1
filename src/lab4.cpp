#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>
#include <chrono>
#include <cmath>

/**
 * @class DeadReckoningNode
 * @brief A node that implements dead reckoning for a robot.
 * 
 * This node subscribes to odometry messages, publishes noisy odometry, and commands 
 * the robot's velocity based on the given parameters.
 */
class DeadReckoningNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the DeadReckoningNode.
     */
    DeadReckoningNode() : Node("dead_reckoning_node"), gen_(rd_()), noise_dist_(0.0, 0.01)
    {
        linear_speed_ = this->declare_parameter("linear_speed", 0.2); // linear speed
        angular_speed_ = this->declare_parameter("angular_speed", 0.0); // angular speed
        distance_ = this->declare_parameter("distance", 2.0); // distance to travel
        direction_ = this->declare_parameter("direction", "forward"); // movement speed

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // velocity command publisher
        odom_noisy_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_noisy", 10); // noisy odometry publisher
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DeadReckoningNode::odom_callback, this, std::placeholders::_1)); // odometry subscriber

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DeadReckoningNode::timer_callback, this)); // timer for periodic updates

        initialized_ = false;
        last_time_ = this->now();
    }

private:
    /**
     * @brief Callback function for odometry messages.
     * 
     * Initializes the pose of the robot when the first odometry message is received.
     * 
     * @param msg The odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!initialized_)
        {
            initial_pose_x_ = msg->pose.pose.position.x; // initial x position
            initial_pose_y_ = msg->pose.pose.position.y; // inital y position

            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch; // Temporary variables to hold roll and pitch
            m.getRPY(roll, pitch, initial_pose_theta_); // get initial orientation

            current_pose_x_ = initial_pose_x_; // current x position
            current_pose_y_ = initial_pose_y_; // current y position
            current_pose_theta_ = initial_pose_theta_; // current orientation

            initialized_ = true;
        }
    }

    /**
     * @brief Timer callback for updating the robot's pose and publishing commands.
     */
    void timer_callback()
    {
        if (!initialized_)
            return;

        auto current_time = this->now();
        auto delta_time = (current_time - last_time_).seconds(); // time difference

        double delta_x = linear_speed_ * delta_time * cos(angular_speed_ * delta_time); // change in x
        double delta_y = linear_speed_ * delta_time * sin(angular_speed_ * delta_time); // change in y
        if (direction_ == "backward")
        {
            delta_x = -delta_x;
            delta_y = -delta_y;
        }

        current_pose_x_ += delta_x; // update current x
        current_pose_y_ += delta_y; // update current y
        current_pose_theta_ += angular_speed_ * delta_time; // update current orientation
        
        // write me code that wrap theta angle

        // current_pose_theta_ = normalize_angle(current_pose_theta_);

        double traveled_distance = std::hypot(current_pose_x_ - initial_pose_x_, current_pose_y_ - initial_pose_y_);

        if (traveled_distance >= distance_)
        {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); // stop the robot
        }
        else
        {
            publish_noisy_odometry(current_time); // publish noisy odometry
            publish_cmd_vel(); // publish velocity command
        }

        last_time_ = current_time;
    }

    /**
     * @brief Publishes noisy odometry data.
     * 
     * @param current_time The current timestamp.
     */
    void publish_noisy_odometry(const rclcpp::Time& current_time)
    {
        current_pose_x_ = current_pose_x_ + noise_dist_(gen_); // add noise to x
        current_pose_y_ = current_pose_y_ + noise_dist_(gen_); // add noise to y
        current_pose_theta_ = current_pose_theta_ + noise_dist_(gen_); // add noise to theta

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_pose_theta_); // create quaternion from current orientation

        nav_msgs::msg::Odometry odom_noisy;
        odom_noisy.header.stamp = current_time; // timestamp
        odom_noisy.header.frame_id = "odom";
        odom_noisy.child_frame_id = "base_link";
        odom_noisy.pose.pose.position.x = current_pose_x_; // current x position
        odom_noisy.pose.pose.position.y = current_pose_y_; // current y position
        odom_noisy.pose.pose.orientation = tf2::toMsg(q); // current orientation
        odom_noisy_pub_->publish(odom_noisy); // publish noisy odometry
    }

    /**
     * @brief Publishes the velocity command based on the robot's direction.
     */
    void publish_cmd_vel()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = (direction_ == "forward") ? linear_speed_ : -linear_speed_; // set linear speed
        cmd_vel.angular.z = angular_speed_; // set angular speed
        cmd_vel_pub_->publish(cmd_vel); // publish command velocity
    }

    /**
     * @brief Normalizes the angle to the range [0, 360).
     * 
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    double normalize_angle(double angle)
    {
        while (angle >= 360.0) angle -= 360.0; // wrap around for angles >= 360
        while (angle < 0.0) angle += 360.0; // wrap around for angles < 0
        return angle;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; //!< velocity command publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_noisy_pub_; //!< noisy odometry publisher
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; //!< odometry subscriber
    rclcpp::TimerBase::SharedPtr timer_; //!< timer for periodic updates

    double linear_speed_; //!< linear speed
    double angular_speed_; //!< angular speed
    double distance_; //!< distance to travel
    std::string direction_; //!< movement direction

    bool initialized_; //!< initialization flag
    double initial_pose_x_, initial_pose_y_, initial_pose_theta_; //!< initial pose variables
    double current_pose_x_, current_pose_y_, current_pose_theta_; //!< current pose variables
    rclcpp::Time last_time_; //!< last time update

    std::random_device rd_; //!< random device for noise generation
    std::mt19937 gen_; //!< random number generator
    std::normal_distribution<> noise_dist_; //!< noise distribution
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeadReckoningNode>());
    rclcpp::shutdown();
    return 0;
}