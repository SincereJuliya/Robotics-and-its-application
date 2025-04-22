#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <graph_for_task_planner_msg/msg/graph.hpp>
#include <graph_for_task_planner_msg/msg/point.hpp>
#include <graph_for_task_planner_msg/msg/edge.hpp> // The message type for the robot's path

#include "../include/robotPlanning/multiPointMarkovDubins.hpp" // Include your MultiPointMarkovDubins header

class MultiPointMarkovDubins {
public:
    MultiPointMarkovDubins() {}

    // Method to generate a path from a list of points
    std::vector<curve> generateMultiPointDubinsPath(const std::vector<geometry_msgs::msg::Point>& points) {
        std::vector<curve> path;

        if (points.size() < 2) {
            return path; // Return empty path
        }

        for (size_t i = 0; i < points.size() - 1; ++i) {
            double x0 = points[i].x;
            double y0 = points[i].y;
            double th0 = 0.0; // Initial orientation

            double xf = points[i + 1].x;
            double yf = points[i + 1].y;
            double thf = 0.0; // Final orientation

            // Assuming dubinsShortestPath returns a compatible curve object
            curve curve = dubinsShortestPath(x0, y0, th0, xf, yf, thf);
            path.push_back(curve);
        }

        return path;
    }
};

class MotionPlanner : public rclcpp::Node
{
public:
    MotionPlanner() : Node("motionPlanner")
    {
        path_subscriber_ = this->create_subscription<graph_for_task_planner_msg::msg::Graph>(
            "taskPlannerGraph", 10, std::bind(&MotionPlanner::graphCallback, this, std::placeholders::_1));

        follow_path_publisher_ = this->create_publisher<graph_for_task_planner_msg::msg::Edge>("followPath", 10);

        current_node_index_ = 0;
    }

private:
    void graphCallback(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg)
    {
        std::vector<geometry_msgs::msg::Point> task_path;

        // Assuming the field is 'vertex_points' or similar
        for (const auto& point : msg->vertices) {
            task_path.push_back(point);
        }

        MultiPointMarkovDubins multi_dubins;
        std::vector<curve> processed_path = multi_dubins.generateMultiPointDubinsPath(task_path);

        for (current_node_index_ = 0; current_node_index_ < processed_path.size(); ++current_node_index_) {
            bool crash_detected = false;

            if (crash_detected) {
                RCLCPP_WARN(this->get_logger(), "Crash detected! Reverting to previous node.");
                current_node_index_ = std::max(0, current_node_index_ - 1);
                break;
            }

            graph_for_task_planner_msg::msg::Edge follow_path_msg;
            follow_path_msg.vertex_points.clear();  // Correct field for points in Edge
            follow_path_msg.vertex_points.push_back(processed_path[current_node_index_].i); // Ensure proper point access

            follow_path_publisher_->publish(follow_path_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    rclcpp::Subscription<graph_for_task_planner_msg::msg::Graph>::SharedPtr path_subscriber_;
    rclcpp::Publisher<graph_for_task_planner_msg::msg::Edge>::SharedPtr follow_path_publisher_;

    int current_node_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPlanner>());
    rclcpp::shutdown();
    return 0;
}
