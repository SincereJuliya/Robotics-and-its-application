#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "graph_for_task_planner_msg/msg/graph.hpp" 
#include "graph_for_task_planner_msg/msg/edge.hpp"
#include "graph_for_task_planner_msg/msg/point.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include "../include/robotPlanning/IMapGenerator.hpp"
#include "../include/robotPlanning/point.hpp" 
#include "../include/robotPlanning/graph.hpp" 

class SampleBasedMapGenerator : public rclcpp::Node, IMapGenerator {
private:
    std::vector<Point> gates_;
    std::vector<Point> borders_;
    std::vector<std::vector<Point>> obstacles_;

    bool data_received_;  // Flag to ensure map is generated only once
    
    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu gates", msg->poses.size());
        gates_.clear();
        for (const auto& pose : msg->poses) {
            gates_.emplace_back(pose.position.x, pose.position.y);
        }
        checkAndGenerateMap();
    }
    
    void bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received PolygonStamped with %zu points", msg->polygon.points.size());
        borders_.clear();
        for (const auto& point : msg->polygon.points) {
            borders_.emplace_back(point.x, point.y);
        }
        checkAndGenerateMap();
    }
    
    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received ObstacleArray with %zu obstacles", msg->obstacles.size());
        obstacles_.clear();
        for (const auto& obstacle : msg->obstacles) {
            std::vector<Point> obstacle_points;
            for (const auto& point : obstacle.polygon.points) {
                obstacle_points.emplace_back(point.x, point.y);
            }
            obstacles_.push_back(obstacle_points);
        }
        checkAndGenerateMap();
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_sub_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_sub_;
    rclcpp::Publisher<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_pub_;

public:
    Graph G;
    
    SampleBasedMapGenerator() : Node("mapGenerator"), data_received_(false) {
        gates_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gates", 10, std::bind(&SampleBasedMapGenerator::gatesCallback, this, std::placeholders::_1));
        
        borders_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/borders", 10, std::bind(&SampleBasedMapGenerator::bordersCallback, this, std::placeholders::_1));
        
        obstacles_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", 10, std::bind(&SampleBasedMapGenerator::obstaclesCallback, this, std::placeholders::_1));
        
        graph_pub_ = this->create_publisher<graph_for_task_planner_msg::msg::Graph>("generatedGraph", 10);
        RCLCPP_INFO(this->get_logger(), "SampleBasedMapGenerator node initialized.");
    }
    
    void checkAndGenerateMap() {
        // Check if all data is received (gates, borders, obstacles)
        if (!data_received_ && !gates_.empty() && !borders_.empty() && !obstacles_.empty()) {
            data_received_ = true; // Flag as data received
            RCLCPP_INFO(this->get_logger(), "All data received. Generating map...");
            toGenerateMap(4);  // Generate map once
            publishGraph();    // Publish the graph
        }
    }

    virtual Graph toGenerateMap(int interactions) override {
        if (gates_.empty() || borders_.empty() || obstacles_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Gates, borders, or obstacles not received!");
            return G; // Return empty graph
        }

        int count = 0;
        Point initP{0, 0};

        while (count < interactions) {
            Point newP = getRandomPoint(initP, getSearchRadius());

            if (isItInObstacle(newP)) {
                continue;
            }
            count++;

            if (G.getVertices().empty()) {
                G.addEdge(initP, newP);  // Ensure at least one valid point
            } else {
                Point nearestP = toFindNearest(G, newP);
                G.addEdge(nearestP, newP);
            }

            if (isReachedGate(newP)) {
                return G;
            }
        }
        return G;
    }

    void publishGraph() {
        if (G.getVertices().empty()) {
            RCLCPP_WARN(this->get_logger(), "Graph is empty! Not publishing.");
            return;
        }
        
        graph_for_task_planner_msg::msg::Graph graph_msg = G.toROSMsg();
        graph_pub_->publish(graph_msg);
        RCLCPP_INFO(this->get_logger(), "Graph message published.");
    }
    
    float getSearchRadius() { return 10.0f; }
    
    Point getRandomPoint(Point initP, float radius) {
        bool validPoint = false;
        float x = initP.getX();
        float y = initP.getY();

        // Keep generating random points until a valid one is found (inside the borders)
        while (!validPoint) {
            x = getRandomPosition(initP.getX(), radius);
            y = getRandomPosition(initP.getY(), radius);
            Point newP{x, y};

            // Check if the point is inside the borders and not inside any obstacles
            if (isInsideBorder(newP) && !isItInObstacle(newP)) {
                validPoint = true;
            }
        }
        return Point{x, y};
    }

    bool isInsideBorder(Point p) {
        // Use a point-in-polygon algorithm (e.g., ray-casting) to check if point p is inside the borders
        int n = borders_.size();
        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if ((borders_[i].getY() > p.getY()) != (borders_[j].getY() > p.getY()) &&
                (p.getX() < (borders_[j].getX() - borders_[i].getX()) * (p.getY() - borders_[i].getY()) / (borders_[j].getY() - borders_[i].getY()) + borders_[i].getX())) {
                inside = !inside;
            }
        }
        return inside;
    }
    
    float getRandomPosition(float middle, float r) {
        return (middle - r) + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * r)));
    }
    
    Point toFindNearest(Graph &g, Point p) {
        if (g.getVertices().empty()) {
            RCLCPP_WARN(this->get_logger(), "Graph is empty! Returning a default point.");
            return Point(0, 0); // Default safe value
        }

        Point nearest;
        auto it = g.getVertices().begin(); 
        float minD = toComputeDistance(it->getX(), it->getY(), p.getX(), p.getY());
        nearest = *it;  // Ensure nearest is initialized
        ++it;

        while (it != g.getVertices().end()) {
            float currentD = toComputeDistance(it->getX(), it->getY(), p.getX(), p.getY());
            if (currentD < minD) {
                minD = currentD;
                nearest = *it;
            }
            ++it;
        }
        return nearest;
    }

    float toComputeDistance(float x0, float y0, float x, float y) {
        return ((x - x0) * (x - x0) + (y - y0) * (y - y0));
    }
    
    bool isReachedGate(Point p) {
        for (const auto& gate : gates_) {
            if (toComputeDistance(gate.getX(), gate.getY(), p.getX(), p.getY()) < 1.0) {
                return true;
            }
        }
        return false;
    }
    
    bool isItInObstacle(Point p) {
        for (const auto& obstacle : obstacles_) {
            for (const auto& point : obstacle) {
                if (toComputeDistance(point.getX(), point.getY(), p.getX(), p.getY()) < 1.0) {
                    return true;
                }
            }
        }
        return false;
    }

    void spin() {
        rclcpp::spin(this->get_node_base_interface());
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SampleBasedMapGenerator>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}
