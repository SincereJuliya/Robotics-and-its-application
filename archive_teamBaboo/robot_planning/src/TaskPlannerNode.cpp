#include "../include/robotPlanning/TaskPlannerNode.hpp"

/**
 * @class TaskPlannerNode
 * @brief A ROS2 node responsible for task planning in a robotic system.
 *
 * This node subscribes to various topics to gather information about the environment,
 * such as gates, obstacles, victims, and borders. It also communicates with the motion
 * planner service to validate paths.
 *
 * Subscriptions:
 * - `/gates` (geometry_msgs::msg::PoseArray): Receives the positions of gates.
 * - `/shelfino/amcl_pose` (geometry_msgs::msg::PoseWithCovarianceStamped): Receives the robot's initial pose.
 * - `/generated_graph` (graph_for_task_planner_msg::msg::Graph): Receives the generated graph for task planning.
 * - `/victims` (obstacles_msgs::msg::ObstacleArrayMsg): Receives information about victims in the environment.
 * - `/obstacles` (obstacles_msgs::msg::ObstacleArrayMsg): Receives information about obstacles in the environment.
 * - `/borders` (geometry_msgs::msg::PolygonStamped): Receives the borders of the operational area.
 *
 * Service Clients:
 * - `validate_path` (motion_planner_msgs::srv::ValidatePath): Used to validate a generated path with the motion planner->
 */
TaskPlannerNode::TaskPlannerNode()
    : rclcpp::Node("taskPlannerNode")
{
    auto qos = get_transient_qos();

    /* ---------------------------------------------------------------------------------------- */
    /*                                        subscriptions                                     */
    /* ---------------------------------------------------------------------------------------- */

    graph_subscriber_ = this->create_subscription<graph_for_task_planner_msg::msg::Graph>(
        "/generated_graph", qos,
        std::bind(&TaskPlannerNode::graphCallback, this, std::placeholders::_1));

    init_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino/amcl_pose", qos,
        std::bind(&TaskPlannerNode::startCallback, this, std::placeholders::_1));

    gates_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gates", qos,
        std::bind(&TaskPlannerNode::gatesCallback, this, std::placeholders::_1));

    borders_subscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/borders", qos,
        std::bind(&TaskPlannerNode::bordersCallback, this, std::placeholders::_1));

    obstacles_subscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", qos,
        std::bind(&TaskPlannerNode::obstaclesCallback, this, std::placeholders::_1));

    victims_subscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/victims", qos,
        std::bind(&TaskPlannerNode::victimsCallback, this, std::placeholders::_1));

    victims_timeout_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/victims_timeout", qos,
        std::bind(&TaskPlannerNode::victimsTimeoutCallback, this, std::placeholders::_1));

    /* ---------------------------------------------------------------------------------------- */
    /*          service to generate a path till its not validated by Motion Planner             */
    /* ---------------------------------------------------------------------------------------- */
    motion_planner_client_ = this->create_client<motion_planner_msgs::srv::ValidatePath>(
        "validate_path");

    RCLCPP_INFO(this->get_logger(), "TaskPlannerNode is ready.");
}

/**
 * @brief Creates and returns a QoS (Quality of Service) profile with transient local durability.
 *
 * This function generates a QoS profile with the specified depth, reliable reliability,
 * and transient local durability. Transient local durability ensures that messages are
 * stored by the middleware for late-joining subscribers.
 *
 * @param depth The depth of the QoS history (default is 10). This determines the number
 *              of messages to store in the history.
 * @return rclcpp::QoS A QoS profile configured with the specified settings.
 */
rclcpp::QoS TaskPlannerNode::get_transient_qos(size_t depth)
{
    rclcpp::QoS qos(depth);
    qos.reliable();
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    return qos;
}
/* ----------------------------------------------------------------------------------------------- */
/*                                                callbacks                                        */
/* ----------------------------------------------------------------------------------------------- */
/**
 * @brief Callback function to handle the reception of gate poses.
 *
 * This function processes a PoseArray message containing the positions and orientations
 * of gates. If the gates have already been received, the function logs a warning and
 * ignores the new message. Otherwise, it clears the existing gates, extracts the
 * positions and orientations from the message, and stores them in the internal data
 * structures. Once the gates are successfully received, it triggers an attempt to
 * generate a path.
 *
 * @param msg Shared pointer to the PoseArray message containing gate poses.
 */
void TaskPlannerNode::gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (gates_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Gates already received, ignoring new message.");
        return; // Ignore if gates are already set
    }
    gates_.clear();
    for (const auto &pose : msg->poses)
    {
        gates_.emplace_back(pose.position.x, pose.position.y);
        th_gates_.push_back(tf2::getYaw(pose.orientation));
    }
    gates_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received %zu gates.", gates_.size());
    attemptGenerate(); // Start path generation attempt
}

/**
 * @brief Callback function to handle the reception of the start pose.
 *
 * This function is triggered when a new start pose message is received. It sets the start pose
 * for the task planner if it has not already been received. If a start pose has already been
 * set, the function logs a warning and ignores the new message. Once the start pose is set,
 * it initiates an attempt to generate a path.
 *
 * @param msg A shared pointer to the received PoseWithCovarianceStamped message containing
 *            the start pose information.
 */
void TaskPlannerNode::startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (start_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Start pose already received, ignoring new message.");
        return; // Ignore if start pose is already set
    }
    start = Point{msg->pose.pose.position.x, msg->pose.pose.position.y};
    start_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received start pose: (%.2f, %.2f)", start.getX(), start.getY());
    attemptGenerate(); // Start path generation attempt
}

/// @brief
/// @param msg
/**
 * @brief Callback function to process the received graph message and initialize the planner.
 *
 * This function is triggered when a new graph message is received. It processes the vertices
 * and edges from the message to construct a graph representation. If a graph has already
 * been received, the function logs a warning and ignores the new message. Once the graph
 * is constructed, it initializes an A* Greedy planner instance and attempts to generate a path.
 *
 * @param msg Shared pointer to the received graph message of type graph_for_task_planner_msg::msg::Graph.
 *
 * The graph message contains:
 * - A list of vertices, each represented by x and y coordinates.
 * - A list of edges, each defined by a start point and an end point with x and y coordinates.
 *
 * @note This function ensures that only the first received graph is processed. Subsequent
 * messages are ignored to prevent overwriting the existing graph.
 */
void TaskPlannerNode::graphCallback(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg)
{
    if (graph_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Graph already received, ignoring new message.");
        return; // Ignore if graph is already set
    }

    graph_.clear();
    for (const auto &v : msg->vertices)
        graph_.addVertice(Point(v.x, v.y));

    for (const auto &e : msg->edges)
        graph_.addEdge(Point(e.start_point.x, e.start_point.y), Point(e.end_point.x, e.end_point.y));

    graph_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received graph with %zu vertices and %zu edges.",
                msg->vertices.size(), msg->edges.size());

    // Create a new AStarGreedy planner instance
    planner = std::make_unique<AStarGreedy>(graph_);

    attemptGenerate(); // Start path generation attempt
}

/**
 * @brief Callback function to process received victim obstacle messages.
 * 
 * This function processes an incoming ObstacleArrayMsg to extract victim information.
 * It clears any previously stored victims, constructs Victim objects from the received
 * obstacles, and stores them in the victims_ list. If victims have already been received,
 * the function logs a warning and ignores the new message.
 * 
 * @param msg Shared pointer to the received ObstacleArrayMsg containing obstacle data.
 */
void TaskPlannerNode::victimsCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    if (victims_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Victims already received, ignoring new message.");
        return; // Ignore if victims are already set
    }

    victims_.clear();
    for (const auto &obstacle : msg->obstacles)
    {
        // Construct a Victim object using x, y, and radius
        if (!obstacle.polygon.points.empty())
        {
            Victim v(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
            victims_.push_back(v);
        }
    }
    victims_received_ = !victims_.empty();
    RCLCPP_INFO(this->get_logger(), "Received %zu victims.", victims_.size());
    attemptGenerate(); // Start path generation attempt
}

/**
 * @brief Callback function to handle incoming obstacle messages.
 *
 * This function processes a message containing an array of obstacles and updates
 * the internal obstacle list. If obstacles have already been received, the function
 * logs a warning and ignores the new message. Otherwise, it clears the current
 * obstacle list, parses the incoming obstacle data, and stores it in the internal
 * structure. After processing, it attempts to generate a plan using the updated
 * obstacle information.
 *
 * @param msg Shared pointer to the incoming obstacle array message.
 */
void TaskPlannerNode::obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    if (obstacles_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Obstacles already received, ignoring new message.");
        return; // Ignore if obstacles are already set
    }

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

/**
 * @brief Callback function to handle the reception of border data.
 *
 * This function processes a received PolygonStamped message containing border points.
 * If borders have already been received, the function logs a warning and ignores the new message.
 * Otherwise, it clears the current borders, populates them with the new points from the message,
 * and sets the borders as received. Finally, it attempts to generate a plan using the new borders.
 *
 * @param msg A shared pointer to the received PolygonStamped message containing the border points.
 */
void TaskPlannerNode::bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    if (borders_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Borders already received, ignoring new message.");
        return; // Ignore if Borders are already set
    }

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
 * @brief Callback function to handle the victims timeout message.
 *
 * This function is triggered when a new victims timeout message is received.
 * If a timeout has already been received, the new message is ignored, and a
 * warning is logged. Otherwise, the timeout value is set to the received value
 * (or a default of 40 if the received value is non-positive), and the
 * attemptGenerate() function is called.
 *
 * @param msg A shared pointer to the received Int32 message containing the
 * victims timeout value.
 */
void TaskPlannerNode::victimsTimeoutCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (victimsTimeoutReceived_)
    {
        RCLCPP_WARN(this->get_logger(), "Victims timeout already received, ignoring new message.");
        return; // Ignore if victims timeout is already set
    }

    victimsTimeout_ = msg->data > 0 ? msg->data : 40; // Default to 40 if not set
    victimsTimeoutReceived_ = true;
    RCLCPP_INFO(this->get_logger(), "Received victims_timeout: %d", victimsTimeout_);
    attemptGenerate();
}

/* ----------------------------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------------------------- */
/*                attempt to generate the path if everything already come                          */
/* ----------------------------------------------------------------------------------------------- */
/**
 * @brief Attempts to generate a path for the task planner->
 *
 * This function checks various conditions to determine if path generation
 * can proceed. If the path has already been validated, or if the maximum
 * number of retry attempts has been exceeded, the function will return early.
 * It also ensures that all required data (start, gates, graph, victims,
 * obstacles, borders) has been received before proceeding with path generation.
 *
 * If all conditions are met, it generates a path and validates it asynchronously.
 *
 * @note The function logs the status of the process at various stages.
 */
void TaskPlannerNode::attemptGenerate()
{
    if (path_validated_)
    {
        RCLCPP_INFO(this->get_logger(), "Path already validated.");
        return;
    }

    if (attempt_count_ > MAX_ATTEMPTS)
    {
        RCLCPP_ERROR(this->get_logger(), "Exceeded max retry attempts.");
        return;
    }

    if (!gates_received_ || !start_received_ || !graph_received_ || !victims_received_ || !obstacles_received_ || !borders_received_ || !victimsTimeoutReceived_ || data_generated_)
    {
        RCLCPP_INFO(get_logger(), "Waiting for all data to be received...");
        RCLCPP_INFO(get_logger(), "Start: %s, Gates: %s, Graph: %s, Victims: %s, Obstacles: %s, Borders: %s, Victims Timeout: %s",
                    start_received_ ? "yes" : "no",
                    gates_received_ ? "yes" : "no",
                    graph_received_ ? "yes" : "no",
                    victims_received_ ? "yes" : "no",
                    obstacles_received_ ? "yes" : "no",
                    borders_received_ ? "yes" : "no",
                    victimsTimeoutReceived_ ? "yes" : "no");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Generating path on attempt %d", attempt_count_);

    std::vector<Point> path = generatePath(attempt_count_); // Generation!! 

    validatePath(path); // Use the async version with callback
}

/* ----------------------------------------------------------------------------------------------- */
/**
 * @brief Processes the failed segments by adding a penalty to the edges in the planner->
 *
 * This function iterates through the list of failed segments and applies a large penalty
 * to the corresponding edges in the A* planner to discourage their selection in future
 * planning attempts. After processing, the list of failed segments is cleared.
 *
 * @param planner Reference to the AStarGreedy planner instance where penalties are applied.
 *
 * The penalty is applied using the `addEdgePenaltyClosest` method of the planner, which
 * modifies the cost of traversing the edges between the specified points. A log message
 * is generated for each penalty added, indicating the coordinates of the penalized edge.
 */
void TaskPlannerNode::processFailedSegments(AStarGreedy &planner, std::vector<std::pair<Point, Point>> &segments, int penaltyValue)
{
    for (const auto &segment : segments)
    {
        const Point &p1 = segment.first;
        const Point &p2 = segment.second;

        planner.addEdgePenaltyClosest(p1, p2, penaltyValue);

        RCLCPP_INFO(this->get_logger(), "Added penalty for edge between (%.5f, %.5f) and (%.5f, %.5f)",
                    p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    segments.clear(); // Clear after processing
}

void processSlowSegments(AStarGreedy &planner,
                         const std::vector<std::pair<Point, Point>> &slowSegments,
                         int attempt)
{
    std::vector<Victim> visitedVictims;

    for (const auto &seg : slowSegments)
    {
        const Point &a = seg.first;
        const Point &b = seg.second;

        planner.addVictimPenaltyFromSegment(a, b, visitedVictims);
    }

    for (auto &v : visitedVictims)
    {
        Point vp(v.x, v.y);
        planner.addVictimPenalty(vp, attempt, v.radius);
    }
}

/* ----------------------------------------------------------------------------------------------- */
/*                          generate the path using A* Greedy algorithm                            */
/* ----------------------------------------------------------------------------------------------- */
/**
 * @brief Generates a path using the A* Greedy algorithm based on the provided attempt number.
 *
 * This function integrates the start point, goal point, victims, and other necessary data into
 * the graph and then uses the A* Greedy planner to compute the best path. The function ensures
 * that all required data is received before proceeding with the path generation.
 *
 * @param attempt The attempt number, which determines the goal point and other parameters.
 *                - If attempt <= 2, the first gate is used as the goal.
 *                - Otherwise, the last gate is used as the goal.
 *
 * @return std::vector<Point> The generated path as a vector of points. If the required data
 *         is not available, an empty path is returned.
 *
 * @note The function logs the progress of the path generation process, including the integration
 *       of start and goal points, victims, and the initialization of the A* Greedy planner->
 *
 * @details
 * - The function checks if all required data (start, gates, graph, victims, obstacles, borders)
 *   has been received before proceeding.
 * - The graph is copied to avoid modifying the original graph.
 * - The start point is integrated into the graph, and the goal point is determined based on the
 *   attempt number.
 * - Victims are integrated into the graph to account for their influence on the path planning.
 * - A time limit is set for the A* Greedy planner based on the attempt number.
 * - If there are failed segments from previous attempts, they are processed and excluded from
 *   the planning process.
 * - The function logs the collected value after running the planner->
 *
 * @warning If the required data is not available, the function logs the missing data and returns
 *          an empty path.
 */
std::vector<Point> TaskPlannerNode::generatePath(int attempt)
{
    if (start_received_ && gates_received_ && graph_received_ && victims_received_ && obstacles_received_ && borders_received_ && victimsTimeoutReceived_) // Ensure all data is available
    {
        Graph graph = graph_; // Create a copy of the graph bc which gates depend on the attempt number, i dont want to add both

        /* ------------------------------------------------------------------------------------------------------------------------------ */
        /*                       yes this part is not nice but readable - to create a new func is too hard to find/read                   */
        /* ------------------------------------------------------------------------------------------------------------------------------ */
        Point start = this->start;
        connectVisibleEdges(graph, start);
        RCLCPP_INFO(this->get_logger(), "Start point integrated into the graph.");

        Point goal = attempt <= 2 ? gates_.front() : gates_.back();
        th_current_goal_ = attempt <= 2 ? th_gates_.front() : th_gates_.back();
        connectVisibleEdges(graph, goal);
        RCLCPP_INFO(this->get_logger(), "Goal point integrated into the graph.");

        integrateVictimsIntoGraph(graph, victims_);
        RCLCPP_INFO(this->get_logger(), "Victims integrated into the graph.");

        /* ------------------------------------------------------------------------------------------------------------------------------ */

        RCLCPP_INFO(this->get_logger(), "Planning A* from (%.2f, %.2f) to (%.2f, %.2f)", start.getX(), start.getY(), goal.getX(), goal.getY());

        double timeLimit = victimsTimeout_;
        RCLCPP_INFO(this->get_logger(), "Time limit for A* Greedy: %.2f seconds", timeLimit);

        planner = std::make_unique<AStarGreedy>(graph, victims_, start, goal, timeLimit, obstacles_, borders_);

        // print failed segments if any
        if (!failedSegments_.empty())
        {
            processFailedSegments(*planner, failedSegments_, 1000); // add a penalty of 1000 to failed segments - didnt make define
            planner->buildMetaGraph();
            RCLCPP_WARN(this->get_logger(), "Processing %zu failed segments from previous attempts.", failedSegments_.size());
        }

        // print slow segments if any
        if (!slowSegments_.empty())
        {
            processSlowSegments(*planner, slowSegments_, timeLimit);
            planner->buildMetaGraph(); // Rebuild with updated penalties
            RCLCPP_WARN(this->get_logger(), "Processing %zu slow segments from previous attempts.", slowSegments_.size());
        }

        double valueCollected = 0.0;
        std::vector<Point> bestPath = planner->run(valueCollected, attempt);

        std::cout << "Collected value: " << valueCollected << std::endl;
        
        std::cout << "--------------------------------------------------------------------" << std::endl;
        std::cout << "--------------------------------------------------------------------" << std::endl;

        return bestPath;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Waiting for all data to be received: "
                                  "start: %s, gates: %s, graph: %s, victims: %s, obstacles: %s, borders: %s",
                    start_received_ ? "yes" : "no",
                    gates_received_ ? "yes" : "no",
                    graph_received_ ? "yes" : "no",
                    victims_received_ ? "yes" : "no",
                    obstacles_received_ ? "yes" : "no",
                    borders_received_ ? "yes" : "no");
        return {}; // Return an empty path if data is not ready
    }
}

/* ------------------------------------------------------------------------------------------ */
/*                          validate the path using Motion Planner service                    */
/* ------------------------------------------------------------------------------------------ */
/**
 * @brief Validates a given path by sending it to the motion planner service.
 *
 * This function sends the provided path to the `ValidatePath` service to check its validity.
 * If the service is unavailable, an error is logged, and the function returns immediately.
 * If the path is valid, it updates the internal state to indicate success. If the path is invalid,
 * it logs the failed segments and retries path generation up to a maximum of 20 attempts.
 *
 * @param path A vector of Points representing the path to be validated.
 *
 * The function performs the following steps:
 * - Waits for the `ValidatePath` service to become available.
 * - Sends the path to the service for validation.
 * - If the path is valid:
 *   - Logs a success message.
 *   - Updates internal flags (`data_generated_` and `path_validated_`) to true.
 * - If the path is invalid:
 *   - Logs a warning message with the attempt count.
 *   - Logs details of any failed segments, including their start and end points.
 *   - Retries path generation if the attempt count is less than 20.
 *   - Logs an error if the maximum number of attempts is reached.
 *
 * @note This function uses asynchronous communication with the `ValidatePath` service.
 */
void TaskPlannerNode::validatePath(const std::vector<Point> &path)
{
    if (!motion_planner_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "ValidatePath service unavailable.");
        return;
    }

    auto request = std::make_shared<motion_planner_msgs::srv::ValidatePath::Request>();
    request->path = convertToPathMessage(path);

    // to start the timer for service
    rclcpp::Time start_time = this->now();

    auto future_result = motion_planner_client_->async_send_request(request,
                                                                    [this, path](rclcpp::Client<motion_planner_msgs::srv::ValidatePath>::SharedFuture result)
                                                                    {
                                                                        auto res = result.get();

                                                                        if (res->valid)
                                                                        {
                                                                            RCLCPP_INFO(this->get_logger(), "Generated path is valid.");
                                                                            data_generated_ = true;
                                                                            path_validated_ = true;
                                                                        }
                                                                        else
                                                                        {
                                                                            RCLCPP_WARN(this->get_logger(), "Generated path is invalid (attempt %d)", attempt_count_);
                                                                            data_generated_ = false;

                                                                            // Print failed segments if any
                                                                            if (!res->failed_segments.empty())
                                                                            {
                                                                                for (const auto &seg : res->failed_segments)
                                                                                {
                                                                                    RCLCPP_WARN(this->get_logger(), "Failed segment: Start (%.5f, %.5f), End (%.5f, %.5f)",
                                                                                                seg.start.x, seg.start.y, seg.end.x, seg.end.y);
                                                                                    failedSegments_.emplace_back(Point(seg.start.x, seg.start.y), Point(seg.end.x, seg.end.y));
                                                                                }
                                                                            }

                                                                            // Print slow segments if any
                                                                            if (!res->slow_segments.empty())
                                                                            {
                                                                                for (const auto &seg : res->slow_segments)
                                                                                {
                                                                                    RCLCPP_WARN(this->get_logger(), "Slow segment: Start (%.5f, %.5f), End (%.5f, %.5f)",
                                                                                                seg.start.x, seg.start.y, seg.end.x, seg.end.y);
                                                                                    slowSegments_.emplace_back(Point(seg.start.x, seg.start.y), Point(seg.end.x, seg.end.y));
                                                                                }
                                                                            }

                                                                            if (attempt_count_ < MAX_ATTEMPTS)
                                                                            {
                                                                                RCLCPP_INFO(this->get_logger(), "Retrying...");
                                                                                attempt_count_ += 1;
                                                                                attemptGenerate(); // triggers a new path generation
                                                                            }
                                                                            else
                                                                            {
                                                                                RCLCPP_ERROR(this->get_logger(), "Failed to generate a valid path after 20 attempts.");
                                                                            }
                                                                        }
                                                                    });

    // to log the time taken for graph generation
    rclcpp::Time end_time = this->now();

    RCLCPP_INFO(get_logger(), "TASKPLANNERNODE: Service was working for %f seconds.", (end_time - start_time).seconds());
}

/* ------------------------------------------------------------------------------------------ */
/*                          integrate the point into the graph                                */
/* ------------------------------------------------------------------------------------------ */
/**
 * @brief Checks if the line segment between two points collides with any obstacle or border.
 *
 * This function interpolates points along the line segment between the given points `p1` and `p2`.
 * It checks each interpolated point to determine if it is too close to an obstacle or inside an obstacle.
 * The number of interpolated points is determined based on the distance between `p1` and `p2`,
 * with 3 samples per unit of distance.
 *
 * @param p1 The starting point of the line segment.
 * @param p2 The ending point of the line segment.
 * @return true If any interpolated point collides with an obstacle or border.
 * @return false If no collision is detected along the line segment.
 */
bool TaskPlannerNode::collidesWithObstacle(const Point &p1, const Point &p2) const
{
    double distance = std::hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());

    // 3 samples per one unit of distance
    const double samplesPerUnit = 3.0;
    int numSamples = std::max(1, static_cast<int>(distance * samplesPerUnit));

    for (int i = 0; i <= numSamples; ++i)
    {
        double t = static_cast<double>(i) / numSamples;
        Point interp(
            p1.getX() + t * (p2.getX() - p1.getX()),
            p1.getY() + t * (p2.getY() - p1.getY()));

        for (const auto &obs : obstacles_)
        {
            if (obs.isTooCloseToObstacle(interp, COLLIDES_OBSTACLE_ASTAR) || obs.isInsideObstacle(interp))
            {
                return true;
            }
        }
    }

    return false; // No collision detected
}

/**
 * @brief Connects visible edges from a new point to existing vertices in the graph.
 *
 * This function attempts to connect a new point to all existing vertices in the graph
 * while adhering to specific constraints to avoid invalid connections. It skips connections
 * based on the following conditions:
 * - If the new point is the start point and the vertex is a victim point or a gate.
 * - If the new point is a victim point and the vertex is the start point, another victim point, or a gate.
 * - If the new point is a gate and the vertex is a victim point or the start point.
 *
 * If none of the above conditions are met, and there is no existing edge between the points
 * and no collision with obstacles or borders, an edge is added between the new point and the vertex.
 *
 * @param graph The graph to which edges will be added.
 * @param newP The new point to connect to existing vertices in the graph.
 */
void TaskPlannerNode::connectVisibleEdges(Graph &graph, const Point &newP) const
{
    std::vector<Point> victims_points;
    for (const auto &victim : victims_)
    {
        victims_points.emplace_back(victim.x, victim.y);
    }

    const auto &verts = graph.getVertices();
    for (const auto &v : verts)
    {
        if (v == newP)
            continue;

        if (newP == start && (std::find(victims_points.begin(), victims_points.end(), v) != victims_points.end() || std::find(gates_.begin(), gates_.end(), v) != gates_.end()))
        {
            std::cout << "Skipping connection from start to victim point: " << v.toString() << std::endl;
            continue;
        }

        if (std::find(victims_points.begin(), victims_points.end(), newP) != victims_points.end()

            && (std::find(victims_points.begin(), victims_points.end(), v) != victims_points.end() || v == start || std::find(gates_.begin(), gates_.end(), v) != gates_.end())

        )
        {
            std::cout << "Skipping connection from victim point to start: " << v.toString() << std::endl;
            continue;
        }

        if (std::find(gates_.begin(), gates_.end(), newP) != gates_.end() && (std::find(victims_points.begin(), victims_points.end(), v) != victims_points.end() || v == start))
        {
            std::cout << "Skipping connection from gate to victim point: " << v.toString() << std::endl;
            continue;
        }

        if (!graph.edgeExists(newP, v) && !collidesWithObstacle(newP, v))
        {
            graph.addEdge(newP, v);
        }
    }
}

/**
 * @brief Integrates a list of victims into the given graph by connecting visible edges.
 *
 * This function iterates through a list of victims, creates a point for each victim's
 * coordinates, and connects the point to the graph using visible edges.
 *
 * @param graph The graph to which the victims will be integrated.
 * @param victims A vector of victims, each containing x and y coordinates.
 */
void TaskPlannerNode::integrateVictimsIntoGraph(Graph &graph, const std::vector<Victim> &victims)
{
    for (const auto &victim : victims)
    {
        Point victimPoint(victim.x, victim.y);
        connectVisibleEdges(graph, victimPoint);
    }
}

/* ------------------------------------------------------------------------------------------ */
/*                          convert the path to a ROS message format                          */
/* ------------------------------------------------------------------------------------------ */
/**
 * @brief Converts a vector of points into a PoseArray message.
 * @param path A vector of Point objects representing the path.
 * @return A geometry_msgs::msg::PoseArray message containing the converted poses.
 */
geometry_msgs::msg::PoseArray TaskPlannerNode::convertToPathMessage(const std::vector<Point> &path)
{
    geometry_msgs::msg::PoseArray msg;
    for (const auto &p : path)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.getX();
        pose.position.y = p.getY();
        pose.position.z = 0.0;

        // Convert yaw angle to quaternion
        double yaw = th_current_goal_; // radians
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);              // Yaw-only rotation
        pose.orientation = tf2::toMsg(q); // Convert to geometry_msgs::Quaternion

        msg.poses.push_back(pose);
    }
    return msg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskPlannerNode>());
    rclcpp::shutdown();
    return 0;
}