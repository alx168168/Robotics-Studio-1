#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector() : Node("cylinder_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        // Subscribe to LaserScan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::laser_callback, this, std::placeholders::_1));

        // Publisher for RViz Marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/detected_cylinder", 10);
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Parameters for the cylinder
        const double cylinder_diameter = 0.30; // 30 cm
        const double cylinder_radius = cylinder_diameter / 2.0;

        // Find the closest points in LaserScan data (rough cylinder detection)
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double range = msg->ranges[i];
            if (std::isfinite(range) && range < msg->range_max && range > msg->range_min)
            {
                double angle = msg->angle_min + i * msg->angle_increment;

                // Basic condition to detect a potential cylinder
                if (range <= cylinder_radius) // Example condition for detecting the cylinder shape
                {
                    RCLCPP_INFO(this->get_logger(), "Cylinder detected at range: %f, angle: %f", range, angle);

                    // Convert LaserScan coordinates to the 'base_link' frame
                    geometry_msgs::msg::PointStamped laser_point;
                    laser_point.header.frame_id = msg->header.frame_id;
                    laser_point.header.stamp = msg->header.stamp;
                    laser_point.point.x = range * cos(angle);
                    laser_point.point.y = range * sin(angle);
                    laser_point.point.z = 0.0;

                    // Transform the point to the map frame
                    geometry_msgs::msg::PointStamped map_point;
                    try
                    {
                        tf_buffer_.transform(laser_point, map_point, "map");
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                        continue;
                    }

                    // Publish a marker to visualize the cylinder in RViz
                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = this->now();
                    marker.ns = "cylinder";
                    marker.id = 0;
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
                    marker.action = visualization_msgs::msg::Marker::ADD;

                    marker.pose.position.x = map_point.point.x;
                    marker.pose.position.y = map_point.point.y;
                    marker.pose.position.z = 0.15; // Assume cylinder height
                    marker.pose.orientation.w = 1.0;

                    marker.scale.x = cylinder_diameter;
                    marker.scale.y = cylinder_diameter;
                    marker.scale.z = 0.30; // Cylinder height

                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;

                    marker_pub_->publish(marker);

                    break; // Exit loop after detecting one cylinder (simplified)
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
