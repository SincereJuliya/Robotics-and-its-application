#include <rclcpp/rclcpp.hpp>
#include <graph_for_task_planner_msg/msg/graph.hpp>
#include <graph_for_task_planner_msg/msg/point.hpp>
#include <graph_for_task_planner_msg/msg/edge.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/IPathPlanner.hpp"
#include "../include/robotPlanning/AStar.hpp"  
#include "motion_planner_msgs/srv/validate_path.hpp"


class TaskPlannerNode : public rclcpp::Node {
public:
    TaskPlannerNode() : Node("taskPlannerNode") {
        // Create a subscriber for "gates" topic
        /* gates_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "gates", rclcpp::QoS(10), std::bind(&MapGeneratorNode::gatesCallback, this, std::placeholders::_1));
 */
        // Create a subscriber for "graph" topic
        graph_subscriber_ = this->create_subscription<graph_for_task_planner_msg::msg::Graph>(
            "generated_graph", 10, std::bind(&TaskPlannerNode::graphCallback, this, std::placeholders::_1));

        // Create a publisher for "taskPlannerPath" topic
        path_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("task_planner_path", 10);

        motion_planner_client_ = this->create_client<motion_planner_msgs::srv::ValidatePath>(
            "validate_path");  // Service client to validate path

        RCLCPP_INFO(this->get_logger(), "A* Node initialized and waiting for graph messages.");
    }

private:
    std::unique_ptr<IPathPlanner> planner_;
    /* vector<Point> goals; */
/* 
    rclcpp::Subscription<graph_for_task_planner_msg::msg::PoseArray>::SharedPtr gates_subscriber_; */
    rclcpp::Subscription<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_publisher_;
    rclcpp::Client<motion_planner_msgs::srv::ValidatePath>::SharedPtr motion_planner_client_;

    Graph graph_;

  /*   void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
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

    } */


    void graphCallback(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received graph with %ld vertices and %ld edges",
                    msg->vertices.size(), msg->edges.size());

        // Clear existing graph data
        graph_.clear();
        // Process graph and generate a path (same as before)
        std::vector<Point> path = generatePath(msg);  // Placeholder function for path generation
        publishPath(path);

        /* // Convert vertices
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
        } */

        /* // Example points for A* search
        Point start = graph_.getVertices().front();
        Point goal = graph_.getVertices().back();  // Use the first vertex as goal

        RCLCPP_INFO(this->get_logger(), "Start and goal for A*: (%f, %f) -> (%f, %f)",
                        start.getX(), start.getY(), goal.getX(), goal.getY());

        //graph_.addEdge(Point{4.25, 3.375}, Point{4.25, 1.875});
        
        planner_ = std::make_unique<AStar>(graph_, start, goal); // Select algorithm
        std::vector<Point> path = planner_->findPath(); // Use the algorithm
        publishPath(path); */

        /* ----------------------------------------------------------------------------------- */
        // Call the MotionPlanner to validate the path
        if (!motion_planner_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner service is not available.");
            return;
        }

        auto request = std::make_shared<motion_planner_msgs::srv::ValidatePath::Request>();
        request->path = convertToPathMessage(path);  // Convert the path to a format acceptable by MotionPlanner

        auto result_future = motion_planner_client_->async_send_request(request);
        result_future.wait_for(std::chrono::seconds(5));

        if (result_future.get()->valid) {
            RCLCPP_INFO(this->get_logger(), "Path is valid, proceeding with MotionPlanner.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Path is invalid, regenerating path.");
            // Optionally, you can regenerate the path here or notify the user
        }
    }

    // Function to convert the vector of points into the ROS path message format
    geometry_msgs::msg::PoseArray convertToPathMessage(const std::vector<Point>& path) {
        geometry_msgs::msg::PoseArray path_msg;
        for (const auto& point : path) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.getX();
            pose.position.y = point.getY();
            path_msg.poses.push_back(pose);
        }
        return path_msg;
    }

    // Your existing method to generate the path
    std::vector<Point> generatePath(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg) {
        
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
        
        planner_ = std::make_unique<AStar>(graph_, start, goal); // Select algorithm
        std::vector<Point> path = planner_->findPath(); // Use the algorithm

        return path;
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