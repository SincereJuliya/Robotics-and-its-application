#include "../include/robotPlanning/MapGeneratorNode.hpp"

/* ---------------------------------------------------------------------------------------- */
/*  node for 2 map generations - SampleBasedMapGenerator + CellDecompositionMapGenerator    */
/* ---------------------------------------------------------------------------------------- */
/**
 * @brief Constructor for the MapGeneratorNode class.
 * 
 * This node is responsible for generating a map using a specified strategy
 * and managing subscriptions and publications for map-related data.
 * 
 * The constructor initializes the map generation strategy, sets up
 * subscribers for various topics, and creates a publisher for the generated
 * graph. It also logs the readiness of the node with the selected strategy.
 * 
 * @details
 * - The map generation strategy can be specified via the "strategy" parameter.
 *   - "sample": Uses SampleBasedMapGenerator.
 *   - "cell" (default): Uses CellDecompositionMapGenerator.
 * - Subscriptions:
 *   - `/shelfino/amcl_pose`: Receives the robot's pose with covariance.
 *   - `/gates`: Receives the positions of gates as a PoseArray.
 *   - `/borders`: Receives the map borders as a PolygonStamped.
 *   - `/obstacles`: Receives the obstacles as an ObstacleArrayMsg.
 * - Publications:
 *   - `/generated_graph`: Publishes the generated graph for task planning.
 * 
 * @note The QoS settings for the subscriptions and publications are configured
 *       using a transient QoS profile.
 */
MapGeneratorNode::MapGeneratorNode() : Node("mapGeneratorNode")
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

/**
 * @brief Creates and returns a QoS (Quality of Service) profile with transient local durability.
 *
 * This function generates a QoS profile with the specified depth, sets it to be reliable,
 * and configures it with transient local durability. Transient local durability ensures
 * that messages are stored by the publisher and are available to late-joining subscribers.
 *
 * @param depth The depth of the QoS profile, which determines the size of the message queue.
 * @return rclcpp::QoS A QoS profile configured with the specified depth, reliability, and transient local durability.
 */
rclcpp::QoS MapGeneratorNode::get_transient_qos(size_t depth)
{
    rclcpp::QoS qos(depth);
    qos.reliable();
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    return qos;
}

/* ------------------------------------------------------------------------------------------ */
/*                                      callbacks                                             */
/* ------------------------------------------------------------------------------------------ */
/**
 * @brief Callback function to handle the reception of the start pose.
 * 
 * This function is triggered when a PoseWithCovarianceStamped message is received.
 * It extracts the x and y coordinates from the pose and stores them as the start point.
 * Additionally, it sets a flag indicating that the start pose has been received and logs
 * the received coordinates. Finally, it attempts to generate the map.
 * 
 * @param msg A shared pointer to the received PoseWithCovarianceStamped message containing
 *            the start pose information.
 */
void MapGeneratorNode::startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    start = Point{msg->pose.pose.position.x, msg->pose.pose.position.y};
    start_received_ = true;
    //RCLCPP_INFO(this->get_logger(), "Received start pose: (%.2f, %.2f)", start.getX(), start.getY());
    attemptGenerate();
}

/**
 * @brief Callback function to process received gate positions.
 *
 * This function is triggered when a new PoseArray message is received. It clears the existing
 * gate positions, extracts the x and y coordinates from each pose in the message, and stores
 * them in the `gates_` container. Once the gates are processed, it sets the `gates_received_`
 * flag to true and logs the number of gates received. Finally, it attempts to generate a map
 * or perform further processing by calling `attemptGenerate()`.
 *
 * @param msg A shared pointer to the received PoseArray message containing gate positions.
 */
void MapGeneratorNode::gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
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

/**
 * @brief Callback function to handle incoming border data.
 *
 * This function is triggered when a new `PolygonStamped` message is received.
 * It processes the polygon points from the message, clears the existing borders,
 * and updates the internal borders list with the new points. Once the borders
 * are updated, it sets the `borders_received_` flag to true and logs the number
 * of points received. Finally, it attempts to generate the map using the updated
 * borders.
 *
 * @param msg A shared pointer to the received `geometry_msgs::msg::PolygonStamped` message
 *            containing the polygon data for the borders.
 */
void MapGeneratorNode::bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
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

/**
 * @brief Callback function to process received obstacle data.
 *
 * This function is triggered when a new message containing obstacle data is received.
 * It clears the current list of obstacles and populates it with the new data from the message.
 * Each obstacle is constructed using its radius and a vector of points representing its polygon.
 * After processing the obstacles, it sets the `obstacles_received_` flag to true and logs the
 * number of obstacles received. Finally, it attempts to generate the map or perform further
 * processing by calling `attemptGenerate()`.
 *
 * @param msg Shared pointer to the received ObstacleArrayMsg containing obstacle data.
 */
void MapGeneratorNode::obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
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
/**
 * @brief Attempts to generate a graph if all required data has been received.
 * 
 * This function checks if all necessary data (start, gates, borders, and obstacles) 
 * has been received and if the graph data has not already been generated. If all 
 * conditions are met, it proceeds to generate the graph using the provided data. 
 * The time taken for graph generation is logged, and the generated graph is published.
 * 
 * If any required data is missing, it logs the current status of the received data.
 * 
 * @note The function ensures that graph generation is performed only once by 
 *       setting the `data_generated_` flag to true after successful generation.
 */
void MapGeneratorNode::attemptGenerate()
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
/**
 * @brief Publishes a graph to a ROS topic.
 *
 * This method converts the given graph into a ROS message format and publishes it
 * using the graph publisher. It also logs the number of vertices in the graph.
 *
 * @param graph The graph to be published. It is expected to be an instance of the
 *              Graph class, which provides a method to convert itself to a ROS message.
 */
void MapGeneratorNode::publishGraph(const Graph &graph)
{
    graph_for_task_planner_msg::msg::Graph msg = graph.toROSMsg();
    graph_publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published graph with %zu vertices.", graph.getVertices().size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
