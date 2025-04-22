#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include <chrono>
#include <vector>
#include <iostream>
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/obstacles.hpp"
#include "../include/robotPlanning/graph.hpp"
#include "../include/robotPlanning/IMapGenerator.hpp"

using namespace std::chrono_literals;

// DataRetriver Class: Responsible for subscribing to topics and storing data
class DataRetriver : public rclcpp::Node {
public:
    DataRetriver() : Node("dataRetriver") {
        this->bordersTopic_ = "/borders";
        this->bordersSubscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            bordersTopic_, 10, std::bind(&DataRetriver::borderCallback, this, std::placeholders::_1));

        this->gatesTopic_ = "/gates";
        this->gatesSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            gatesTopic_, 10, std::bind(&DataRetriver::gatesCallback, this, std::placeholders::_1));

        this->obstaclesTopic_ = "/obstacles";
        this->obstaclesSubscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            obstaclesTopic_, 10, std::bind(&DataRetriver::obstaclesCallback, this, std::placeholders::_1));
    }

    const std::vector<Point>& getBorders() const { return borders_; }
    const std::vector<Point>& getGates() const { return gates_; }
    const std::vector<Obstacle>& getObstacles() const { return obstacles_; }

    // + publisher

private:
    void borderCallback(const geometry_msgs::msg::PolygonStamped msg) {
        for (const auto& point : msg.polygon.points) {
            borders_.emplace_back(point.x, point.y);
        }
    }

    void gatesCallback(const geometry_msgs::msg::PoseArray msg) {
        for (const auto& pose : msg.poses) {
            gates_.emplace_back(pose.position.x, pose.position.y);
        }
    }

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg msg) {
        for (const auto& obstacle : msg.obstacles) {
            std::vector<Point> vertices;
            for (const auto& point : obstacle.polygon.points) {
                vertices.emplace_back(point.x, point.y);
            }
            obstacles_.emplace_back(obstacle.radius, vertices);
        }
    }

    std::string bordersTopic_;
    std::string gatesTopic_;
    std::string obstaclesTopic_;

    std::vector<Point> borders_;
    std::vector<Point> gates_;
    std::vector<Obstacle> obstacles_;

    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr bordersSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber_;
};

// SampleBasedMapGenerator Class: Generates the map using data from DataRetriver
class SampleBasedMapGenerator : public rclcpp::Node, public IMapGenerator {
public:
    SampleBasedMapGenerator(std::shared_ptr<DataRetriver> dataRetriever)
        : Node("mapGenerator"), dataRetriever_(dataRetriever) {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("generatedMap", 10);
    }

    virtual Graph toGenerateMap(int iterations) override {
        int count = 0;
        Point initP{0, 0};
        auto obstacles = dataRetriever_->getObstacles();
        auto& gates = dataRetriever_->getGates();

        while (count < iterations) {
            Point newP = getRandomPoint(initP, getSearchRadius());

            if (isItInObstacle(newP, obstacles)) {
                continue;
            }
            count++;

            Point nearestP = toFindNearest(graph_, newP);
            graph_.addEdge(nearestP, newP);

            if (isReachedGate(newP, gates)) {
                return graph_;
            }
        }
        return graph_;
    }

    void publishGraphAsPoseArray() {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = rclcpp::Clock().now();
        pose_array.header.frame_id = "map";

        for (const auto& vertex : graph_.getVertices()) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = vertex.getX();
            pose.position.y = vertex.getY();
            pose.position.z = 0.0;

            pose.orientation.w = 1.0;
            pose_array.poses.push_back(pose);
        }
        publisher_->publish(pose_array);
    }

private:
    float getSearchRadius() { return 10.0f; }

    Point getRandomPoint(Point initP, float radius) {
        float x = initP.getX() + (rand() % static_cast<int>(2 * radius)) - radius;
        float y = initP.getY() + (rand() % static_cast<int>(2 * radius)) - radius;
        return Point{x, y};
    }

    bool isItInObstacle( Point& p, std::vector<Obstacle>& obstacles) {
        for ( auto obstacle : obstacles) {
            if (obstacle.isInsideObstacle(p)) {
                return true;
            }
        }
        return false;
    }

    bool isReachedGate(const Point& p, const std::vector<Point>& gates) {
        return false;
    }

    float toComputeDistance(float x0, float y0, float x, float y)
    {
            return ((x - x0) * (x - x0) + (y - y0) * (y - y0));
    }

    Point toFindNearest(Graph& g, Point p) {
        Point nearest;
        float minD = std::numeric_limits<float>::max();

        for (const auto& vertex : g.getVertices()) {
            float currentD = toComputeDistance(vertex.getX(),vertex.getY(),p.getX(),p.getY());
            if (currentD < minD) {
                minD = currentD;
                nearest = vertex;
            }
        }
        return nearest;
    }

    std::shared_ptr<DataRetriver> dataRetriever_;
    Graph graph_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto dataRetriever = std::make_shared<DataRetriver>();
    auto mapGenerator = std::make_shared<SampleBasedMapGenerator>(dataRetriever);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(dataRetriever);
    executor.add_node(mapGenerator);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
