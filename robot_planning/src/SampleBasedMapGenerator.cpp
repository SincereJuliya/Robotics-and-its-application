#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "stdio.h"
#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include "../include/robotPlanning/IMapGenerator.hpp"
#include "../include/robotPlanning/point.hpp" 
#include "../include/robotPlanning/graph.hpp" 

class SampleBasedMapGenerator : public rclcpp::Node, IMapGenerator  {
private:
    // Callback function to retrieve the border data received as PolygonStamped message
    void dataCallback(const geometry_msgs::msg::PoseArray msg) {
        RCLCPP_INFO(this->get_logger(), "DataCallback received a PoseArray message with %zu points", msg.poses.size());
        for (size_t i = 0; i < msg.poses.size(); ++i) 
            {
                std::cout << "Gates: '" << i << "'\n";
                std::cout << "Received: x: '" << msg.poses[i].position.x << "', y: '" << msg.poses[i].position.y << ", whith orientation: " << msg.poses[i].orientation.w << " + " << msg.poses[i].orientation.x << "i + " << msg.poses[i].orientation.y << "j + " << msg.poses[i].orientation.z << "k \n";
                // Store the received points in the gate vector
                //G.push_back(Point(msg.poses[i].position.x, msg.poses[i].position.y));
            }
    }
    std::string dataTopic_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr dataSubscriber_;

    // ROS 2 node and publisher
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher;

public:
    Graph G;

    // Constructor
    SampleBasedMapGenerator(): Node("mapGenerator")
    {
        // Set topic name
        //this->dataTopic_ = "/borders";
        RCLCPP_INFO(this->get_logger(), "Node initialized, subscribing to topic: %s", dataTopic_.c_str());
        // Create subscription to dataTopic_ topic
        //this->dataSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        //    dataTopic_.c_str(), 10, std::bind(&SampleBasedMapGenerator::dataCallback, this, std::placeholders::_1));

        // Create publisher to publish the graph as PoseArray
        // Initialize publisher
        publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("generatedMap", 10);


    }

    // Destructor (no need to shutdown ROS here)
    ~SampleBasedMapGenerator() {}

    // Function to publish the graph as PoseArray
    void publishGraphAsPoseArray()
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = rclcpp::Clock().now();
        pose_array.header.frame_id = "map";  // frame is "map"

        for (const auto &vertex : G.getVertices())
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = vertex.getX();  
            pose.position.y = vertex.getY();  
            pose.position.z = 0.0;  // 2D map

            // Set orientation to identity quaternion (no rotation)
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;

            pose_array.poses.push_back(pose);
        }

        publisher->publish(pose_array);
    }

    virtual Graph toGenerateMap(int interactions) override
    {
        int count = 0;
        Point initP{0, 0};

        while (count < interactions)
        {
            Point newP = getRandomPoint(initP, getSearchRadius());

            if (isItInObstacle(newP))
            {
                continue;
            }
            count++;

            Point nearestP = toFindNearest(G, newP);
            G.addEdge(nearestP, newP);

            if (isReachedGate(newP))
            {
                return G;
            }
        }
        return G;
    }

    // Get the search radius for generating new points (consider nearest obstacle)
    float getSearchRadius()
    {
        // Placeholder for actual logic to compute radius based on obstacles
        // Assume a fixed radius for now, replace with actual logic
        return 10.0f;
    }

    // Get a new random point near the initial point within a given radius
    Point getRandomPoint(Point initP, float radius)
    {
        bool flag = true;
        float x = initP.getX();
        float y = initP.getY();

        while (flag)
        {
            x = getRandomPosition(initP.getX(), radius);
            y = getRandomPosition(initP.getY(), radius);

            flag = !isInsideArea(x - initP.getX(), y - initP.getY(), radius);
        }

        return Point{x, y}; // Return by value instead of dynamically allocated memory
    }

    // Check if the point is inside a circular area
    bool isInsideArea(float x, float y, float r)
    {
        return (toComputeDistance(0, 0, x, y) <= r * r);
    }

    // Generate a random position within a given range
    float getRandomPosition(float middle, float r)
    {
        float random = (middle - r) + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * r)));
        return random;
    }

    // Find the nearest point in the graph to connect the new point
    Point toFindNearest(Graph &g, Point p)
    {
        Point nearest;

        auto it = g.getVertices().begin(); 
        float minD = toComputeDistance(it->getX(), it->getY(), p.getX(), p.getY());
        ++it;

        while (it != g.getVertices().end())
        {
            float currentD = toComputeDistance(it->getX(), it->getY(), p.getX(), p.getY());

            if (currentD < minD)
            {
                minD = currentD;
                nearest = *it;
            }

            ++it;
        }

        return nearest;
    }

    // Compute the squared distance between two points
    float toComputeDistance(float x0, float y0, float x, float y)
    {
        return ((x - x0) * (x - x0) + (y - y0) * (y - y0));
    }

    // Check if the point has reached the gate
    bool isReachedGate(Point p)
    {
        // Placeholder logic for gate detection, replace with actual check
        return false;
    }

    // Check if the point is in an obstacle
    bool isItInObstacle(Point p)
    {
        // Placeholder logic for obstacle detection, replace with actual check
        return false;
    }

    // Spin the ROS node to process callbacks
    void spin()
    {
        rclcpp::spin(node);
    } 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SampleBasedMapGenerator>());
    rclcpp::shutdown();  
    return 0;
}

