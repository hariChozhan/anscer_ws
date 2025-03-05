#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <unordered_map>
#include <fstream>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class WaypointPlanner : public rclcpp::Node {
public:
    WaypointPlanner() : Node("waypoint_planner") {
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        loadWaypoints();
    }

    void navigateToWaypoint(int waypoint_id) {
        if (waypoints_.find(waypoint_id) == waypoints_.end()) {
            RCLCPP_WARN(get_logger(), "Waypoint %d not found!", waypoint_id);
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = waypoints_[waypoint_id];

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = 
            [this](const GoalHandleNav::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Navigation to waypoint succeeded.");
                } else {
                    RCLCPP_WARN(get_logger(), "Navigation to waypoint failed.");
                }
            };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void loadWaypoints() {
        std::ifstream file("waypoints.yaml");
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open waypoints.yaml");
            return;
        }

        int id;
        double x, y, theta;
        while (file >> id >> x >> y >> theta) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation.z = theta;
            waypoints_[id] = pose;
        }

        file.close();
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    std::unordered_map<int, geometry_msgs::msg::PoseStamped> waypoints_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto planner = std::make_shared<WaypointPlanner>();

    // Example: Navigate to waypoint ID 1
    planner->navigateToWaypoint(1);

    rclcpp::spin(planner);
    rclcpp::shutdown();
    return 0;
}
