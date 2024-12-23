#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include <chrono>
#include <vector>
#include <iostream>
#include "../include/robotPlanning/variables.hpp"

using namespace std::chrono_literals;

class DataRetriver : public rclcpp::Node {
public:
    DataRetriver() : Node("dataRetriver") {
        // Set borders topic name
        this->bordersTopic_ = "/borders";
        RCLCPP_INFO(this->get_logger(), "Node initialized, subscribing to topic: %s", bordersTopic_.c_str());
        // Create subscription to border topic to receive PolygonStamped messages
        this->bordersSubscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            bordersTopic_.c_str(), 10, std::bind(&DataRetriver::borderCallback, this, std::placeholders::_1));

        // Set gates topic name
        this->gatesTopic_ = "/gates";
        RCLCPP_INFO(this->get_logger(), "Node for gates initialized, subscribing to topic: %s", gatesTopic_.c_str());
        // Create subscription to receive PolygonStamped messages
        this->gatesSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            gatesTopic_.c_str(), 10, std::bind(&DataRetriver::gatesCallback, this, std::placeholders::_1));

        this->obstaclesTopic_ = "/obstacles";
        RCLCPP_INFO(this->get_logger(), "Node initialized, subscribing to topic: %s", obstaclesTopic_.c_str());
        // Create subscription to receive ObstacleArrayMsg messages
        this->obstaclesSubscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            obstaclesTopic_.c_str(), 10, std::bind(&DataRetriver::obstaclesCallback, this, std::placeholders::_1));
    }

private:
    // Callback function to retrieve the border data received as PolygonStamped message
    void borderCallback(const geometry_msgs::msg::PolygonStamped msg) {
        RCLCPP_INFO(this->get_logger(), "borderCallback received a PolygonStamped message with %zu points", msg.polygon.points.size());

        // Iterate over the points of the polygon
        for (size_t i = 0; i < msg.polygon.points.size(); ++i) {
            // Print the index and the coordinates of each point
            std::cout << "Border: '" << i << "'\n";
            std::cout << "Received: x: '" << msg.polygon.points[i].x << "', y: '" << msg.polygon.points[i].y << "'\n";
            // Store the received points in the border vector
            borders.push_back(Point{msg.polygon.points[i].x, msg.polygon.points[i].y});
        }
    }

    // Callback function to retrieve the gates data received as PoseArray message
    void gatesCallback(const geometry_msgs::msg::PoseArray msg) {
        RCLCPP_INFO(this->get_logger(), "Callback received a PoseArray message with %zu points", msg.poses.size());

        // Iterate over the points of the polygon
        for (size_t i = 0; i < msg.poses.size(); ++i) {
            // Print the index and the coordinates of each point
            std::cout << "Gates: '" << i << "'\n";
            std::cout << "Received: x: '" << msg.poses[i].position.x << "', y: '" << msg.poses[i].position.y << ", whith orientation: " << msg.poses[i].orientation.w << " + " << msg.poses[i].orientation.x << "i + " << msg.poses[i].orientation.y << "j + " << msg.poses[i].orientation.z << "k \n";
            // Store the received points in the gate vector
            gates.push_back(Point{msg.poses[i].position.x, msg.poses[i].position.y});
        }
    }

    // Callback function to retrieve the obstacles data the received ObstacleArrayMsg message
    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg msg) {
        RCLCPP_INFO(this->get_logger(), "Callback received a ObstacleArrayMsg message with %zu obstacles", msg.obstacles.size());

        // Iterate over the obstacles 
        for (size_t i = 0; i < msg.obstacles.size(); ++i) {
            // Check if circle or box to store in the right object
            if(msg.obstacles[i].radius){
                std::cout << "Ostacle: '" << i << "of type circle '\n";
                std::cout << "Received: Cx: '" << msg.obstacles[i].polygon.points[0].x << "', Cy: '" << msg.obstacles[i].polygon.points[0].y << " with radius: " << msg.obstacles[i].radius <<"'\n";
                // Store the received points in a circle struct
                Circle c {{msg.obstacles[i].polygon.points[0].x, msg.obstacles[i].polygon.points[0].y}, msg.obstacles[i].radius};
                circleObstacles.push_back(c);
            }else{
                std::cout << "Ostacle: '" << i << "of type box '\n";
                Square sq;
                for(size_t j=0; j< msg.obstacles[i].polygon.points.size(); j++){
                    std::cout << "Received: x: '" << msg.obstacles[i].polygon.points[j].x << "', y: '" << msg.obstacles[i].polygon.points[j].y << "'\n";
                    sq.obstacle.push_back({msg.obstacles[i].polygon.points[j].x, msg.obstacles[i].polygon.points[j].y});
                }
            }
        }
    }

    std::string bordersTopic_;
    std::string gatesTopic_;
    std::string obstaclesTopic_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr bordersSubscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataRetriver>());
    rclcpp::shutdown();  
    return 0;
}


/* #include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include <chrono>
#include <vector>
#include <iostream>
#include "../include/robotPlanning/variables.hpp"


using namespace std::chrono_literals;

class Border : public rclcpp::Node {
public:
    Border() : Node("border") {
        // Set borders topic name
        this->bordersTopic_ = "/borders";
        RCLCPP_INFO(this->get_logger(), "Node initialized, subscribing to topic: %s", bordersTopic_.c_str());
        // Create subscription to border topic to receive PolygonStamped messages
        this->bordersSubscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            bordersTopic_.c_str(), 10, std::bind(&Border::borderCallback, this, std::placeholders::_1));
    }

private:
    // Callback function to retrieve the border data received as PolygonStamped message
    void borderCallback(const geometry_msgs::msg::PolygonStamped msg) {
        RCLCPP_INFO(this->get_logger(), "borderCallback received a PolygonStamped message with %zu points", msg.polygon.points.size());

        // Iterate over the points of the polygon
        for (size_t i = 0; i < msg.polygon.points.size(); ++i) {
            // Print the index and the coordinates of each point
            std::cout << "Border: '" << i << "'\n";
            std::cout << "Received: x: '" << msg.polygon.points[i].x << "', y: '" << msg.polygon.points[i].y << "'\n";
            // Store the received points in the border vector
            borders.push_back(Point{msg.polygon.points[i].x, msg.polygon.points[i].y});
        }
    }

    std::string bordersTopic_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr bordersSubscriber_;
};

class Gate : public rclcpp::Node {
public:
    Gate() : Node("gate") {
        // Set gates topic name
        this->gatesTopic_ = "/gates";
        RCLCPP_INFO(this->get_logger(), "Node for gates initialized, subscribing to topic: %s", gatesTopic_.c_str());
        // Create subscription to receive PolygonStamped messages
        this->gatesSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            gatesTopic_.c_str(), 10, std::bind(&Gate::gatesCallback, this, std::placeholders::_1));
    }

private:
    // Callback function to retrieve the gates data received as PoseArray message
    void gatesCallback(const geometry_msgs::msg::PoseArray msg) {
        RCLCPP_INFO(this->get_logger(), "Callback received a PoseArray message with %zu points", msg.poses.size());

        // Iterate over the points of the polygon
        for (size_t i = 0; i < msg.poses.size(); ++i) {
            // Print the index and the coordinates of each point
            std::cout << "Gates: '" << i << "'\n";
            std::cout << "Received: x: '" << msg.poses[i].position.x << "', y: '" << msg.poses[i].position.y << ", whith orientation: " << msg.poses[i].orientation.w << " + " << msg.poses[i].orientation.x << "i + " << msg.poses[i].orientation.y << "j + " << msg.poses[i].orientation.z << "k \n";
            // Store the received points in the gate vector
            gates.push_back(Point{msg.poses[i].position.x, msg.poses[i].position.y});
        }
    }

    std::string gatesTopic_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gatesSubscriber_;
};

class Obstacles : public rclcpp::Node {
public:
    Obstacles() : Node("obstacles") {
        this->obstaclesTopic_ = "/obstacles";
        RCLCPP_INFO(this->get_logger(), "Node initialized, subscribing to topic: %s", obstaclesTopic_.c_str());
        // Create subscription to receive ObstacleArrayMsg messages
        this->obstaclesSubscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            obstaclesTopic_.c_str(), 10, std::bind(&Obstacles::obstaclesCallback, this, std::placeholders::_1));
    }

private:
    // Callback function to retrieve the obstacles data the received ObstacleArrayMsg message
    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg msg) {
        RCLCPP_INFO(this->get_logger(), "Callback received a ObstacleArrayMsg message with %zu obstacles", msg.obstacles.size());

        // Iterate over the obstacles 
        for (size_t i = 0; i < msg.obstacles.size(); ++i) {
            // Check if circle or box to store in the right object
            if(msg.obstacles[i].radius){
                std::cout << "Ostacle: '" << i << "of type circle '\n";
                std::cout << "Received: Cx: '" << msg.obstacles[i].polygon.points[0].x << "', Cy: '" << msg.obstacles[i].polygon.points[0].y << " with radius: " << msg.obstacles[i].radius <<"'\n";
                // Store the received points in a circle struct
                Circle c {{msg.obstacles[i].polygon.points[0].x, msg.obstacles[i].polygon.points[0].y}, msg.obstacles[i].radius};
                circleObstacles.push_back(c);
            }else{
                std::cout << "Ostacle: '" << i << "of type box '\n";
                Square sq;
                for(size_t j=0; j< msg.obstacles[i].polygon.points.size(); j++){
                    std::cout << "Received: x: '" << msg.obstacles[i].polygon.points[j].x << "', y: '" << msg.obstacles[i].polygon.points[j].y << "'\n";
                    sq.obstacle.push_back(Point{msg.obstacles[i].polygon.points[j].x, msg.obstacles[i].polygon.points[j].y});
                }
            }
        }
    }

    std::string obstaclesTopic_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstaclesSubscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Border>());
    rclcpp::spin(std::make_shared<Gate>());
    rclcpp::spin(std::make_shared<Obstacles>());
    rclcpp::shutdown();  
    return 0;
} */
