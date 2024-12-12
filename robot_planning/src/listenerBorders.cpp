#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <chrono>
#include <vector>
#include <iostream>

using namespace std::chrono_literals;

// Define the custom Point struct to store x, y values
typedef struct Point {
    double x;
    double y;
} Point;

// Vector to store gate points (assuming 4 points for the polygon)
std::vector<Point> gate(4);

class MyBorders : public rclcpp::Node {
public:
    MyBorders() : Node("myborders") {
        // Set topic name
        this->topic_ = "/borders";
        RCLCPP_INFO(this->get_logger(), "Node initialized, subscribing to topic: %s", topic_.c_str());

        // Create subscription to receive PolygonStamped messages
        this->subscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            topic_.c_str(), 10, std::bind(&MyBorders::callback, this, std::placeholders::_1));
    }

private:
    // Callback function to process the received PolygonStamped message
    void callback(const geometry_msgs::msg::PolygonStamped msg) {
        RCLCPP_INFO(this->get_logger(), "Callback received a PolygonStamped message with %zu points", msg.polygon.points.size());

        // Iterate over the points of the polygon
        for (size_t i = 0; i < msg.polygon.points.size(); ++i) {
            // Print the index and the coordinates of each point
            std::cout << "Border: '" << i << "'\n";
            std::cout << "Received: x: '" << msg.polygon.points[i].x << "', y: '" << msg.polygon.points[i].y << "'\n";

            // Store the received points in the gate vector
            gate[i].x = msg.polygon.points[i].x;
            gate[i].y = msg.polygon.points[i].y;
        }
    }

    std::string topic_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<MyBorders>());  // Spin the node to keep it alive
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}

