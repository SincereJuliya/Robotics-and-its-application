#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
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

class MyGates : public rclcpp::Node {
public:
    MyGates() : Node("Mygates") {
        // Set topic name
        this->topic_ = "/gates";
        RCLCPP_INFO(this->get_logger(), "Node for gates initialized, subscribing to topic: %s", topic_.c_str());

        // Create subscription to receive PolygonStamped messages
        this->subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            topic_.c_str(), 10, std::bind(&MyGates::callback, this, std::placeholders::_1));
    }

private:
    // Callback function to process the received PolygonStamped message
    void callback(const geometry_msgs::msg::PoseArray msg) {
        RCLCPP_INFO(this->get_logger(), "Callback received a PoseArray message with %zu points", msg.poses.size());

        // Iterate over the points of the polygon
        for (size_t i = 0; i < msg.poses.size(); ++i) {
            // Print the index and the coordinates of each point
            std::cout << "Gates: '" << i << "'\n";
            std::cout << "Received: x: '" << msg.poses[i].position.x << "', y: '" << msg.poses[i].position.y << ", whith orientation: " << msg.poses[i].orientation.w << " + " << msg.poses[i].orientation.x << "i + " << msg.poses[i].orientation.y << "j + " << msg.poses[i].orientation.z << "k \n";

            // Store the received points in the gate vector
            gate[i].x = msg.poses[i].position.x;
            gate[i].y = msg.poses[i].position.y;
        }
    }

    std::string topic_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscriber_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<MyGates>());  // Spin the node to keep it alive
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}

//resource reallocation problem