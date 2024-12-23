#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include <chrono>
#include <vector>
#include <iostream>

using namespace std::chrono_literals;

// Define the custom Point struct to store x, y values
typedef struct Point {
    double x;
    double y;
} Point;

typedef struct Circle {
    Point center;
    double radius;
} Circle;

typedef struct Square {
    std::vector<Point> obstacle;
} Square;

typedef struct Obstacle {
    std::vector<Point> vertices;
    double radius;
}Obstacle;

std::vector<Circle> circleObstacles; //maybe better to create classes, in order to store obstacles, where we have circles and boxes as subclasses

class MyObstacles : public rclcpp::Node {
public:
    MyObstacles() : Node("myobstacles") {
        // Set topic name
        this->topic_ = "/obstacles";
        RCLCPP_INFO(this->get_logger(), "Node initialized, subscribing to topic: %s", topic_.c_str());

        // Create subscription to receive ObstacleArrayMsg messages
        this->subscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            topic_.c_str(), 10, std::bind(&MyObstacles::callback, this, std::placeholders::_1));
    }

private:
    // Callback function to process the received ObstacleArrayMsg message
    void callback(const obstacles_msgs::msg::ObstacleArrayMsg msg) {
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

    std::string topic_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<MyObstacles>());  // Spin the node to keep it alive
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}

