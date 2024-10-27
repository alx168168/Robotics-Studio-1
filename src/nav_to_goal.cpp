#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"  // Include for odometry messages
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class TurtlebotNavNode : public rclcpp::Node {
public:
    TurtlebotNavNode()
        : Node("turtlebot_nav_node"), goal_active_(true),
          tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {  // Initialize TF listener
        // Create an action client to send navigation goals
        this->action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // Create a subscription to the odometry topic
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",  // Replace with the actual odometry topic name if different
            10,
            std::bind(&TurtlebotNavNode::odometry_callback, this, std::placeholders::_1)
        );

        // Wait until the action server is available
        while (!this->action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the navigate_to_pose action server...");
        }
        
        // Set and send the goal
        send_goal(1.8, -7.2, 0);  // (x, y, theta)
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;  // Subscription for odometry
    bool goal_active_;  // Flag to track if the goal is active
    tf2_ros::Buffer tf_buffer_;  // Buffer for TF data
    tf2_ros::TransformListener tf_listener_;  // Listener for TF data

    void send_goal(float x, float y, float theta) {
        // Create a new goal
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

        // Set position and orientation
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->get_clock()->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0);
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0);

        // Send the goal
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this, x, y, theta](auto goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server. Coordinates: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
            }
        };

        send_goal_options.result_callback = [this](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                goal_active_ = false;  // Set the flag to false when the goal is succeeded
            } else {
                RCLCPP_ERROR(this->get_logger(), "Goal failed with status: %d", result.code);
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Only print the coordinates if the goal is still active
        if (goal_active_) {
            // Use tf to transform the odometry pose to the map frame
            geometry_msgs::msg::PoseStamped odom_pose;
            odom_pose.header = msg->header;
            odom_pose.pose = msg->pose.pose;

            try {
                // Transform the odometry pose to the map frame
                geometry_msgs::msg::PoseStamped map_pose;
                map_pose = tf_buffer_.transform(odom_pose, "map", tf2::durationFromSec(1.0));

                double x = map_pose.pose.position.x;
                double y = map_pose.pose.position.y;

                // Print the coordinates to the terminal
                RCLCPP_INFO(this->get_logger(), "Current position in map frame: x=%.2f, y=%.2f", x, y);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform warning: %s", ex.what());
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlebotNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
