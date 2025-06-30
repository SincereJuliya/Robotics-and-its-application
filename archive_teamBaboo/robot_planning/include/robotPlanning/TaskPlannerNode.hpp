#pragma once

#include <rclcpp/rclcpp.hpp>
#include <graph_for_task_planner_msg/msg/graph.hpp>
#include <graph_for_task_planner_msg/msg/point.hpp>
#include <graph_for_task_planner_msg/msg/edge.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include <obstacles_msgs/msg/obstacle_array_msg.hpp> 
#include <obstacles_msgs/msg/obstacle_msg.hpp>  

#include "std_msgs/msg/int32.hpp"

#include "point.hpp"
#include "IPathPlanner.hpp"
#include "agreedy.hpp"
#include "victim.hpp"

#include "motion_planner_msgs/srv/validate_path.hpp"

#include <tf2/LinearMath/Quaternion.h>        
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/utils.h>                         

#define VELOCITY 0.26
#define MAX_ATTEMPTS 20
#define COLLIDES_OBSTACLE_ASTAR 0.6f

class TaskPlannerNode : public rclcpp::Node {
public:
    TaskPlannerNode();

    std::unique_ptr<AStarGreedy> planner;

private:
    Graph graph_;
    // default
    int attempt_count_ = 0;
    int victimsTimeout_ = 40; 
    double th_current_goal_ = 0.0;
    Point start;
    std::vector<double> th_gates_;
    std::vector<Point> borders_;
    std::vector<Point> gates_;
    std::vector<Victim> victims_;
    std::vector<Obstacle> obstacles_;
    std::vector<std::pair<Point, Point>> failedSegments_;
    std::vector<std::pair<Point, Point>> slowSegments_;

    bool start_received_ = false;
    bool gates_received_ = false;
    bool graph_received_ = false;
    bool victims_received_ = false;
    bool obstacles_received_ = false;
    bool borders_received_ = false;
    bool victimsTimeoutReceived_ = false;

    bool data_generated_ = false;
    bool path_validated_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_subscriber_;
    rclcpp::Subscription<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victims_subscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr victims_timeout_subscriber_;

    rclcpp::Client<motion_planner_msgs::srv::ValidatePath>::SharedPtr motion_planner_client_;

    rclcpp::QoS get_transient_qos(size_t depth = 10);

    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void graphCallback(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg);
    void victimsCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void victimsTimeoutCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void attemptGenerate();
    void processFailedSegments(AStarGreedy &planner, std::vector<std::pair<Point, Point>>& segments, int penaltyValue);
    std::vector<Point> generatePath(int attempt);
    void validatePath(const std::vector<Point>& path);

    bool collidesWithObstacle(const Point& p1, const Point& p2) const;
    void connectVisibleEdges(Graph& graph, const Point& newP) const;
    void integratePointIntoGraph(Graph& graph, Point& point);
    void integrateVictimsIntoGraph(Graph& graph, const std::vector<Victim>& victims);

    geometry_msgs::msg::PoseArray convertToPathMessage(const std::vector<Point>& path);
};

int main(int argc, char* argv[]);
