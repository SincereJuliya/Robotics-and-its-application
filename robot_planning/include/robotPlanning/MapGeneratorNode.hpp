#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "graph_for_task_planner_msg/msg/graph.hpp"

#include "point.hpp"
#include "SampleBasedMapGenerator.hpp"
#include "CellDecompositionMapGenerator.hpp"
#include "IMapGenerator.hpp"

class MapGeneratorNode : public rclcpp::Node
{
public:
    MapGeneratorNode();

private:
    std::unique_ptr<IMapGenerator> generator_;

    Point start;
    std::vector<Point> gates_;
    std::vector<Point> borders_;
    std::vector<Obstacle> obstacles_;

    bool start_received_ = false;
    bool gates_received_ = false;
    bool borders_received_ = false;
    bool obstacles_received_ = false;
    bool data_generated_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_subscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber_;
    rclcpp::Publisher<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_publisher_;

    rclcpp::QoS get_transient_qos(size_t depth = 10);

    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void attemptGenerate();
    void publishGraph(const Graph &graph);
};

int main(int argc, char* argv[]);
