#include <rclcpp/rclcpp.hpp>
#include <graph_for_task_planner_msg/msg/graph.hpp>
#include <graph_for_task_planner_msg/msg/point.hpp>
#include <graph_for_task_planner_msg/msg/edge.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/IPathPlanner.hpp"
#include "../include/robotPlanning/AStar.hpp"  
#include "motion_planner_msgs/srv/validate_path.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>



class TaskPlannerNode : public rclcpp::Node {
public:
    TaskPlannerNode() : Node("taskPlannerNode") {
        // Create a subscriber for "gates" topic
        gates_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "gates", rclcpp::QoS(10),
            std::bind(&TaskPlannerNode::gatesCallback, this, std::placeholders::_1));
        // Create a subscriber for "init" topic
        init_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/shelfino/amcl_pose", rclcpp::QoS(10),
            std::bind(&TaskPlannerNode::startCallback, this, std::placeholders::_1));
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
    
    std::vector<Point> gates_;
    Point start;

    bool start_received_ = false;
    bool gates_received_ = false;
    bool graph_received_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_subscriber_; 
    rclcpp::Subscription<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_publisher_;
    rclcpp::Client<motion_planner_msgs::srv::ValidatePath>::SharedPtr motion_planner_client_;

    Graph graph_;

    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        gates_.clear();
        RCLCPP_INFO(this->get_logger(), "Received gates with %zu points.", gates_.size());
        for (const auto &pose : msg->poses) {
            gates_.emplace_back(pose.position.x, pose.position.y);
        }
        gates_received_ = true;

        attemptGenerate(); 

    } 

    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received initial pose: (%f, %f)",
                    msg->pose.pose.position.x, msg->pose.pose.position.y);
        start = Point{msg->pose.pose.position.x, msg->pose.pose.position.y};
        start_received_ = true;

        attemptGenerate(); 

    } 

    void graphCallback(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg) {
        graph_.clear();
        RCLCPP_INFO(this->get_logger(), "Received graph with %ld vertices and %ld edges",
                    msg->vertices.size(), msg->edges.size());
        for (const auto& ros_point : msg->vertices) {
            Point p(ros_point.x, ros_point.y);
            graph_.addVertice(p);  // Add the point as a vertex
            RCLCPP_INFO(this->get_logger(), "Added vertex: (%f, %f)", p.getX(), p.getY());
        }
        for (const auto& ros_edge : msg->edges) {
            Point p1(ros_edge.start_point.x, ros_edge.start_point.y);
            Point p2(ros_edge.end_point.x, ros_edge.end_point.y);

            graph_.addEdge(p1, p2);
            RCLCPP_INFO(this->get_logger(), "Added edge: (%f, %f) -> (%f, %f)",
                        p1.getX(), p1.getY(), p2.getX(), p2.getY());
        }
        graph_received_ = true;

        attemptGenerate(); 

    }

    bool validatePath(const std::vector<Point>& path) {
        if (!motion_planner_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner service is not available.");
            return false;
        }

        auto request = std::make_shared<motion_planner_msgs::srv::ValidatePath::Request>();
        request->path = convertToPathMessage(path);

        auto result_future = motion_planner_client_->async_send_request(request);
        if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "MotionPlanner service call failed or timed out.");
            return false;
        }

        if (result_future.get()->valid) {
            RCLCPP_INFO(this->get_logger(), "Path is valid, proceeding.");
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Path is invalid.");
            return false;
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

    void attemptGenerate() {
        if (gates_received_ && start_received_ && graph_received_) {
            RCLCPP_INFO(get_logger(), "All data received; generating path.");
            std::vector<Point> path = generatePath();

            publishPath(path);

            if (!validatePath(path)) {
                RCLCPP_WARN(this->get_logger(), "Initial path invalid, attempting regeneration...");
            }
        }
    }

    // Your existing method to generate the path
    std::vector<Point> generatePath() {
        Point goal = gates_.front(); 

            if (gates_received_) {
                RCLCPP_INFO(this->get_logger(), "Goal added to graph: (%f, %f)", goal.getX(), goal.getY());

                // Now find the nearest point in the graph to the goal and add the edge
                Point nearest_point = graph_.findNearestPoint(goal);  // You would need a method like this in your graph

                graph_.addVertice(goal);  // Add the goal to the graph
                graph_.addEdge(nearest_point, goal);  // Create an edge to the goal

                RCLCPP_INFO(this->get_logger(), "Edge added between nearest point and goal." 
                            " Nearest point: (%f, %f)", nearest_point.getX(), nearest_point.getY());
            }

            if (start_received_) {
                RCLCPP_INFO(this->get_logger(), "Start added to graph: (%f, %f)", start.getX(), start.getY());

                // Now find the nearest point in the graph to the goal and add the edge
                Point nearest_point = graph_.findNearestPoint(start);  // You would need a method like this in your graph

                graph_.addVertice(start);  // Add the goal to the graph
                graph_.addEdge(nearest_point, start);  // Create an edge to the goal

                RCLCPP_INFO(this->get_logger(), "Edge added between nearest point and start." 
                            " Nearest point: (%f, %f)", nearest_point.getX(), nearest_point.getY());
            }

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