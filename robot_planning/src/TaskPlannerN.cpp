#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <graph_for_task_planner_msg/msg/graph.hpp>
#include <graph_for_task_planner_msg/msg/point.hpp>
#include <graph_for_task_planner_msg/msg/edge.hpp>
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/TaskPlanner.hpp"

class AStarNode : public rclcpp::Node {
public:
    AStarNode() : Node("taskPlanner") {
        // Create a subscriber for "graph" topic
        graph_subscriber_ = this->create_subscription<graph_for_task_planner_msg::msg::Graph>(
            "graph", 10, std::bind(&AStarNode::graphCallback, this, std::placeholders::_1));

        // Create a publisher for "followPath" topic
        path_publisher_ = this->create_publisher<std_msgs::msg::String>("followPath", 10);
    }

private:
    rclcpp::Subscription<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_publisher_;
    Graph graph_;

   void graphCallback(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received graph with %ld vertices and %ld edges",
                    msg->vertices.size(), msg->edges.size());

        // Clear existing graph data
        graph_.clear();

        // Convert vertices
        for (const auto& ros_point : msg->vertices) {
            Point p(ros_point.x, ros_point.y);
            graph_.addVertice(p);  // Add the point as a vertex
            RCLCPP_INFO(this->get_logger(), "Added vertex: (%f, %f)", p.getX(), p.getY());
        }

        // Convert edges
        for (const auto& ros_edge : msg->edges) {
            Point p1(ros_edge.start_point.x, ros_edge.start_point.y);
            Point p2(ros_edge.end_point.x, ros_edge.end_point.y);

            graph_.addEdge(p1, p2);
            RCLCPP_INFO(this->get_logger(), "Added edge: (%f, %f) -> (%f, %f)",
                        p1.getX(), p1.getY(), p2.getX(), p2.getY());
        }

        // Example points for A* search
        Point start(0, 0);  
        Point goal(10, 10);  

        // Run A* and publish the result
        AStar astar(graph_, start, goal);
        std::vector<Point> path = astar.findPath();
        publishPath(path);
    }



    // Method to publish the path
    void publishPath(const std::vector<Point>& path) {
        std_msgs::msg::String path_msg;
        
        // Convert the path to a string for easy visualization
        for (const auto& point : path) {
            path_msg.data += "(" + std::to_string(point.getX()) + ", " + std::to_string(point.getY()) + ") ";
        }

        // Publish the path
        path_publisher_->publish(path_msg);
    }
};

// Main function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarNode>());
    rclcpp::shutdown();
    return 0;
} 
