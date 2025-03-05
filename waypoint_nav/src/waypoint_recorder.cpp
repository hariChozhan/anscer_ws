#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "anscer_msgs/msg/waypoint.hpp"
#include "anscer_msgs/srv/waypoint.hpp"
#include <unordered_map>
#include <fstream>
#include <cmath>
#include <mutex>

using std::placeholders::_1;
using std::placeholders::_2;

class WaypointRecorder : public rclcpp::Node
{
public:
    WaypointRecorder() : Node("waypoint_recorder")
    {
        // Load parameters
        declare_parameter("min_distance_threshold", 1.0);
        declare_parameter("min_rotation_threshold", 0.2);
        get_parameter("min_distance_threshold", min_distance_threshold_);
        get_parameter("min_rotation_threshold", min_rotation_threshold_);
        // Create waypoint creation service
        create_waypoint_srv_ = this->create_service<anscer_msgs::srv::Waypoint>(
            "create_waypoint", std::bind(&WaypointRecorder::createWaypointCallback, this, _1, _2));
        // Subscribe to odometry messages
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&WaypointRecorder::odomCallback, this, _1));
    }

private:
    std::mutex waypoint_mutex_;
    rclcpp::Service<anscer_msgs::srv::Waypoint>::SharedPtr create_waypoint_srv_; // Waypoint creation service
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;           // Odometry subscriber
    std::unordered_map<int, geometry_msgs::msg::PoseStamped> waypoints_;         // Waypoints container
    geometry_msgs::msg::PoseStamped last_pose_;                                  // Last pose received from odom
    double min_distance_threshold_;                                              // Minimum distance between waypoints
    double min_rotation_threshold_;                                              // Minimum rotation between waypoints

    // Callback to create a new waypoint
    void createWaypointCallback(const std::shared_ptr<anscer_msgs::srv::Waypoint::Request> request,
                                std::shared_ptr<anscer_msgs::srv::Waypoint::Response> response)
    {
        std::lock_guard<std::mutex> lock(waypoint_mutex_);

        geometry_msgs::msg::PoseStamped new_pose = last_pose_;
        if (waypoints_.empty() || isFarEnough(new_pose) || isRotatedEnough(new_pose))
        {
            waypoints_[request->id] = new_pose;
            saveWaypoints();
            response->success = true;
            RCLCPP_INFO(get_logger(), "Manual Waypoint %d Created.", request->id);
        }
        else
        {
            response->success = false;
            RCLCPP_WARN(get_logger(), "Waypoint too close to the last one created.");
        }
    }

    // Check if the new pose is far enough from the last waypoint
    bool isFarEnough(const geometry_msgs::msg::PoseStamped &new_pose)
    {
        if (waypoints_.empty())
        {
            RCLCPP_WARN(get_logger(), "Waypoint.yaml list is empty");
            return true;
        }

        auto last_wp_it = std::max_element(waypoints_.begin(), waypoints_.end(),
                                           [](const auto &a, const auto &b)
                                           {
                                               return a.first < b.first; // Compare waypoint IDs
                                           });
        if (last_wp_it == waypoints_.end())
        {
            RCLCPP_ERROR(get_logger(), "Waypoint.yaml list is invalid");
            return false;
        }
        const auto &last_wp = last_wp_it->second.pose;
        if (!std::isfinite(last_wp.position.x) || !std::isfinite(last_wp.position.y))
        {
            RCLCPP_ERROR(get_logger(), "Last waypoint contains NaN or Inf values!");
            return false;
        }

        double dx = new_pose.pose.position.x - last_wp.position.x;
        double dy = new_pose.pose.position.y - last_wp.position.y;
        double distance = std::sqrt(dx * dx + dy * dy); // Euclidean distance
        // RCLCPP_INFO(get_logger(), "Distance to last waypoint: %.3f (Threshold: %.3f)", distance, min_distance_threshold_);
        return distance >= min_distance_threshold_;
    }

    // Check if the new pose is rotated enough from the last waypoint
    bool isRotatedEnough(const geometry_msgs::msg::PoseStamped &new_pose)
    {
        if (waypoints_.empty())
        {
            RCLCPP_WARN(get_logger(), "Waypoint.yaml list is empty");
            return true;
        }

        auto last_wp_it = std::max_element(waypoints_.begin(), waypoints_.end(),
                                           [](const auto &a, const auto &b)
                                           {
                                               return a.first < b.first; 
                                           });
        if (last_wp_it == waypoints_.end())
        {
            RCLCPP_ERROR(get_logger(), "Waypoint.yaml list is invalid");
            return false;
        }
        const auto &last_wp = last_wp_it->second.pose;
        if (!std::isfinite(last_wp.position.x) || !std::isfinite(last_wp.position.y))
        {
            RCLCPP_ERROR(get_logger(), "Last waypoint contains NaN or Inf values!");
            return false;
        }

        double dtheta = std::abs(new_pose.pose.orientation.z - last_wp.orientation.z);
        // RCLCPP_INFO(get_logger(), "Distance to last waypoint: %.3f (Threshold: %.3f)", dtheta, min_rotation_threshold_);
        return dtheta >= min_rotation_threshold_;
    }

    // Callback to save the waypoints to a YAML file
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(waypoint_mutex_);

        if (!msg)
        {
            RCLCPP_ERROR(get_logger(), "Received null Odometry message!");
            return;
        }

        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.pose = msg->pose.pose;

        if (waypoints_.empty() || isFarEnough(new_pose) || isRotatedEnough(new_pose))
        {
            int id = waypoints_.size() + 1;
            waypoints_[id] = new_pose;
            saveWaypoints();
            RCLCPP_INFO(get_logger(), "Auto Waypoint %d Created at (%.2f, %.2f, %.2f)",
                        id, new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.orientation.z);
        }
        last_pose_ = new_pose;
    }

    // Save the waypoints to a YAML file
    void saveWaypoints()
    {
        std::ofstream file("waypoints.yaml");
        if (!file)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open waypoints.yaml for writing.");
            return;
        }
        // Write waypoints to the file
        for (const auto &wp : waypoints_)
        {
            file << "- id: " << wp.first << "\n";
            file << "  x: " << wp.second.pose.position.x << "\n";
            file << "  y: " << wp.second.pose.position.y << "\n";
            file << "  theta: " << wp.second.pose.orientation.z << "\n";
        }
        file.close();
        RCLCPP_INFO(get_logger(), "Waypoints saved successfully.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointRecorder>());
    rclcpp::shutdown();
    return 0;
}