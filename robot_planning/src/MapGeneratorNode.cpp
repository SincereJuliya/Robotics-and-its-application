#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "graph_for_task_planner_msg/msg/graph.hpp"
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/SampleBasedMapGenerator.hpp"
#include "../include/robotPlanning/CellDecompositionMapGenerator.hpp"
#include "../include/robotPlanning/IMapGenerator.hpp"

class MapGeneratorNode : public rclcpp::Node {
private:
    std::unique_ptr<IMapGenerator> generator_;

    // Store received data
    std::vector<Point> gates_;
    std::vector<Point> borders_;
    std::vector<Obstacle> obstacles_;

    bool gates_received_ = false;
    bool borders_received_ = false;
    bool obstacles_received_ = false;
    bool data_generated_ = false;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_sub_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_sub_;
    rclcpp::Publisher<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_pub_;

public:
    MapGeneratorNode()
        : Node("mapGeneratorNode")
    {
        std::string strategy = this->declare_parameter<std::string>("strategy", "cell");
        if (strategy == "sample") {
            generator_ = std::make_unique<SampleBasedMapGenerator>();
        } else {
            generator_ = std::make_unique<CellDecompositionMapGenerator>();
        }

        gates_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "gates", rclcpp::QoS(10),
            std::bind(&MapGeneratorNode::gatesCallback, this, std::placeholders::_1));

        borders_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "borders", rclcpp::QoS(10),
            std::bind(&MapGeneratorNode::bordersCallback, this, std::placeholders::_1));

        obstacles_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "obstacles", rclcpp::QoS(10),
            std::bind(&MapGeneratorNode::obstaclesCallback, this, std::placeholders::_1));

        graph_pub_ = this->create_publisher<graph_for_task_planner_msg::msg::Graph>(
            "generated_graph", rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(), "MapGeneratorNode initialized using '%s' strategy.", strategy.c_str());
    }

private:
    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        gates_.clear();
        for (const auto &pose : msg->poses) {
            gates_.emplace_back(pose.position.x, pose.position.y);
        }
        gates_received_ = true;
        attemptGenerate();
    }

    void bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        borders_.clear();
        for (const auto &pt : msg->polygon.points) {
            borders_.emplace_back(pt.x, pt.y);
        }
        borders_received_ = true;
        attemptGenerate();
    }

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        obstacles_.clear();
        for (const auto &obs_msg : msg->obstacles) {
            obstacles_.emplace_back(obs_msg.radius, // or appropriate constructor
                                    [&]() {
                                        std::vector<Point> pts;
                                        for (auto &p : obs_msg.polygon.points)
                                            pts.emplace_back(p.x, p.y);
                                        return pts;
                                    }());
        }
        obstacles_received_ = true;
        attemptGenerate();
    }

    void attemptGenerate() {
        if (gates_received_ && borders_received_ && obstacles_received_ && !data_generated_) {
            generator_->setGates(gates_);
            generator_->setBorders(borders_);
            generator_->setObstacles(obstacles_);

            RCLCPP_INFO(get_logger(), "All data received; generating graph.");
            Graph graph = generator_->generateGraph();

            publishGraph(graph);
            data_generated_ = true;
        }
    }

    void publishGraph(const Graph &graph) {
        graph_for_task_planner_msg::msg::Graph msg = graph.toROSMsg();
        graph_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published graph with %zu vertices.", graph.getVertices().size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS 2

    // Create and spin the MapGeneratorNode
    rclcpp::spin(std::make_shared<MapGeneratorNode>());

    rclcpp::shutdown();  // Shutdown ROS 2 after node stops
    return 0;
}
