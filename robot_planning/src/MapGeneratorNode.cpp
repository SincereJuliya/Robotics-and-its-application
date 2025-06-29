#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "graph_for_task_planner_msg/msg/graph.hpp"
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/SampleBasedMapGenerator.hpp"
#include "../include/robotPlanning/CellDecompositionMapGenerator.hpp"
#include "../include/robotPlanning/IMapGenerator.hpp"

/* ---------------------------------------------------------------------------------------- */
/*  node for 2 map generations - SampleBasedMapGenerator + CellDecompositionMapGenerator    */
/* ---------------------------------------------------------------------------------------- */
class MapGeneratorNode : public rclcpp::Node
{
public:
    MapGeneratorNode() : Node("mapGeneratorNode")
    {
        auto qos = get_transient_qos();

        /* ---------------------------------------------------------------------------------------- */
        /*                     initialization of the method for map generation                      */
        /* ---------------------------------------------------------------------------------------- */
        std::string strategy = this->declare_parameter<std::string>("strategy", "cell");
        if (strategy == "sample")
            generator_ = std::make_unique<SampleBasedMapGenerator>();
        else
            generator_ = std::make_unique<CellDecompositionMapGenerator>();

        /* ---------------------------------------------------------------------------------------- */
        /*                                      subscribers                                         */
        /* ---------------------------------------------------------------------------------------- */
        init_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/shelfino/amcl_pose", qos,
            std::bind(&MapGeneratorNode::startCallback, this, std::placeholders::_1));

        gates_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gates", qos,
            std::bind(&MapGeneratorNode::gatesCallback, this, std::placeholders::_1));

        borders_subscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/borders", qos,
            std::bind(&MapGeneratorNode::bordersCallback, this, std::placeholders::_1));

        obstacles_subscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", qos,
            std::bind(&MapGeneratorNode::obstaclesCallback, this, std::placeholders::_1));

        /* ---------------------------------------------------------------------------------------- */
        /*                                      publish                                             */
        /* ---------------------------------------------------------------------------------------- */
        graph_publisher_ = this->create_publisher<graph_for_task_planner_msg::msg::Graph>(
            "/generated_graph", qos);

        RCLCPP_INFO(this->get_logger(), "MapGeneratorNode is ready using '%s' strategy.", strategy.c_str());
    }

private:
    std::unique_ptr<IMapGenerator> generator_;

    // Store received data
    Point start;
    std::vector<Point> gates_;
    std::vector<Point> borders_;
    std::vector<Obstacle> obstacles_;

    bool start_received_ = false;
    bool gates_received_ = false;
    bool borders_received_ = false;
    bool obstacles_received_ = false;
    bool data_generated_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_subscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber_;
    rclcpp::Publisher<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_publisher_;

    rclcpp::QoS get_transient_qos(size_t depth = 10)
    {
        rclcpp::QoS qos(depth);
        qos.reliable();
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        return qos;
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                                      callbacks                                             */
    /* ------------------------------------------------------------------------------------------ */
    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        start = Point{msg->pose.pose.position.x, msg->pose.pose.position.y};
        start_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received start pose: (%.2f, %.2f)", start.getX(), start.getY());
        attemptGenerate();
    }

    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        gates_.clear();
        for (const auto &pose : msg->poses)
        {
            gates_.emplace_back(pose.position.x, pose.position.y);
        }
        gates_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received %zu gates.", gates_.size());
        attemptGenerate();
    }

    void bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
    {
        borders_.clear();
        for (const auto &pt : msg->polygon.points)
        {
            borders_.emplace_back(pt.x, pt.y);
        }
        borders_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received borders with %zu points.", borders_.size());
        attemptGenerate();
    }

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        obstacles_.clear();
        for (const auto &obs_msg : msg->obstacles)
        {
            obstacles_.emplace_back(obs_msg.radius, // or appropriate constructor
                                    [&]()
                                    {
                                        std::vector<Point> pts;
                                        for (auto &p : obs_msg.polygon.points)
                                            pts.emplace_back(p.x, p.y);
                                        return pts;
                                    }());
        }
        obstacles_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received %zu obstacles.", obstacles_.size());
        attemptGenerate();
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                attempt to generate the graph if everything already come                    */
    /* ------------------------------------------------------------------------------------------ */
    void attemptGenerate()
    {
        if (start_received_ && gates_received_ && borders_received_ && obstacles_received_ && !data_generated_)
        {
            RCLCPP_INFO(get_logger(), "All required data received; generating graph...");
            generator_->setGates(gates_);
            generator_->setBorders(borders_);
            generator_->setObstacles(obstacles_);

            RCLCPP_INFO(get_logger(), "All data received; generating graph.");

            // to start the timer for graph generation
            rclcpp::Time start_time = this->now();

            Graph graph = generator_->generateGraph(start);

            // to log the time taken for graph generation
            rclcpp::Time end_time = this->now();

            RCLCPP_INFO(get_logger(), "MAPGENERATORNODE: Graph generated in %f seconds.", (end_time - start_time).seconds());

            publishGraph(graph);
            data_generated_ = true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Waiting for all data to be received: "
                                      "start: %s, gates: %s, borders: %s, obstacles: %s",
                        start_received_ ? "yes" : "no",
                        gates_received_ ? "yes" : "no",
                        borders_received_ ? "yes" : "no",
                        obstacles_received_ ? "yes" : "no");
        }
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                          publish the generated graph                                       */
    /* ------------------------------------------------------------------------------------------ */
    void publishGraph(const Graph &graph)
    {
        graph_for_task_planner_msg::msg::Graph msg = graph.toROSMsg();
        graph_publisher_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published graph with %zu vertices.", graph.getVertices().size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
