#pragma once

#include <rclcpp/rclcpp.hpp>
#include "motion_planner_msgs/srv/validate_path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "std_msgs/msg/int32.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "point.hpp"
#include "multiPointMarkovDubins.hpp"
#include "obstacles.hpp"

class MotionPlannerNode : public rclcpp::Node
{
public:
    MotionPlannerNode();

private:
    int victimsTimeout_ = 40;
    std::vector<Obstacle> obstacles_;
    std::vector<Point> borders_;
    std::vector<Point> pathFromMsg;
    std::vector<arcVar> dubinsPath;
    std::vector<std::pair<Point, Point>> failedSegments_;
    std::vector<std::pair<Point, Point>> slowSegments_;

    double th_start = 0.0;
    double th_goal_ = 0.0;

    bool data_generated_ = false;
    bool path_received_ = false;
    bool obstacles_received_ = false;
    bool borders_received_ = false;
    bool start_received_ = false;
    bool victimsTimeoutReceived_ = false;

    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr victims_timeout_subscriber_;
    rclcpp::Service<motion_planner_msgs::srv::ValidatePath>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_publisher_;

    rclcpp::QoS get_transient_qos(size_t depth = 10);

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void victimsTimeoutCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void validatePathCallback(
        const std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Request> request,
        std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Response> response);

    void attemptGenerate();
    std::vector<Point> convertPoseArrayToPoints(const geometry_msgs::msg::PoseArray &pose_array_msg);
    motion_planner_msgs::msg::Segment convertToSegmentMsg(const Point &p1, const Point &p2);
    void publishDubinsPath();
};
