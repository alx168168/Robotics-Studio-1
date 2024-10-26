#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TurtlebotNavNode : public rclcpp::Node {
public:
    TurtlebotNavNode()
        : Node("turtlebot_nav_node") {
        // Create an action client to send navigation goals
        this->action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // Wait until the action server is available
        while (!this->action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the navigate_to_pose action server...");
        }
        
        // Set and send the goal
        send_goal(2.0, -6.0, 0);  // (x, y, theta)
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

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
            } else {
                RCLCPP_ERROR(this->get_logger(), "Goal failed with status: %d", result.code);
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlebotNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

