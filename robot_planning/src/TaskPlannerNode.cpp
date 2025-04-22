#include <rclcpp/rclcpp.hpp>
#include <graph_for_task_planner_msg/msg/graph.hpp>
#include <graph_for_task_planner_msg/msg/point.hpp>
#include <graph_for_task_planner_msg/msg/edge.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/IPathPlanner.hpp"
#include "../include/robotPlanning/AStar.hpp"  

class TaskPlannerNode : public rclcpp::Node {
public:
    TaskPlannerNode() : Node("taskPlannerNode") {
        // Create a subscriber for "gates" topic
        gates_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "gates", rclcpp::QoS(10), std::bind(&MapGeneratorNode::gatesCallback, this, std::placeholders::_1));

        // Create a subscriber for "graph" topic
        graph_subscriber_ = this->create_subscription<graph_for_task_planner_msg::msg::Graph>(
            "generated_graph", 10, std::bind(&TaskPlannerNode::graphCallback, this, std::placeholders::_1));

        // Create a publisher for "taskPlannerPath" topic
        path_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("taskPlannerPath", 10);
        RCLCPP_INFO(this->get_logger(), "A* Node initialized and waiting for graph messages.");
    }

private:
    std::unique_ptr<IPathPlanner> planner_;
    vector<Point> goals;

    rclcpp::Subscription<graph_for_task_planner_msg::msg::PoseArray>::SharedPtr gates_subscriber_;
    rclcpp::Subscription<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_publisher_;
    Graph graph_;

    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::vector<Point> gates_;
        
        for (const auto &pose : msg->poses) {
            gates_.emplace_back(pose.position.x, pose.position.y);
        }

        // Calculate middle points between consecutive gates
        for (size_t i = 0; i < gates_.size() - 1; ++i) {
            Point p1 = gates_[i];
            Point p2 = gates_[i + 1];
            
            // Calculate middle point
            float middle_x = (p1.x + p2.x) / 2;
            float middle_y = (p1.y + p2.y) / 2;
            
            // Store middle point
            goals.emplace_back(Point{middle_x, middle_y});
        }

    }


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
        Point start = graph_.getVertices().front();
        Point goal = graph_.getVertices().back();  // Use the first vertex as goal

        RCLCPP_INFO(this->get_logger(), "Start and goal for A*: (%f, %f) -> (%f, %f)",
                        start.getX(), start.getY(), goal.getX(), goal.getY());

        //graph_.addEdge(Point{4.25, 3.375}, Point{4.25, 1.875});
        
        planner_ = std::make_unique<AStar>(graph_, start, goal); // Select algorithm
        std::vector<Point> path = planner_->findPath(); // Use the algorithm
        publishPath(path);
    }

    // Method to publish the path
    void publishPath(const std::vector<Point>& path) {
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path is empty, nothing to publish");
            return;  // If the path is empty, exit early
        }

        geometry_msgs::msg::PoseArray path_msg;
        
        // Convert the path to PoseArray
        for (const auto& point : path) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.getX();
            pose.position.y = point.getY();
            pose.position.z = 0.0;  // Set z to 0 for 2D path
            pose.orientation.w = 1.0;  // Default orientation (no rotation)
            
            path_msg.poses.push_back(pose);
        }

        // Publish the PoseArray on the "taskPlannerPath" topic
        path_publisher_->publish(path_msg);
    }

};

// Main function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskPlannerNode>());
    rclcpp::shutdown();
    return 0;
}