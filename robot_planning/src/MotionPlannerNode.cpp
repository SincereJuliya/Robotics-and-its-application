#include "../include/robotPlanning/MotionPlannerNode.hpp"

/**
 * @brief Constructor for the MotionPlannerNode class.
 * 
 * This initializes the MotionPlannerNode and sets up the necessary ROS 2
 * subscriptions, publishers, and services. The node is responsible for
 * handling motion planning tasks, including receiving obstacle data,
 * border information, initial pose, and victim timeout updates, as well
 * as providing a service to validate paths.
 * 
 * Subscriptions:
 * - `/obstacles`: Subscribes to obstacle data of type `obstacles_msgs::msg::ObstacleArrayMsg`.
 * - `/borders`: Subscribes to border information of type `geometry_msgs::msg::PolygonStamped`.
 * - `/shelfino/amcl_pose`: Subscribes to the robot's initial pose of type `geometry_msgs::msg::PoseWithCovarianceStamped`.
 * - `/victims_timeout`: Subscribes to victim timeout updates of type `std_msgs::msg::Int32`.
 * 
 * Service:
 * - `validate_path`: Provides a service of type `motion_planner_msgs::srv::ValidatePath` 
 *   to validate a given path.
 * 
 * Publisher:
 * - `dubins_plan`: Publishes planned paths of type `geometry_msgs::msg::PoseArray`.
 * 
 * The constructor also logs a message indicating that the MotionPlannerNode is ready.
 */
MotionPlannerNode::MotionPlannerNode() : Node("motionPlannerNode")
{
    auto qos = get_transient_qos();

    /* ---------------------------------------------------------------------------------------- */
    /*                                      subscribers                                         */
    /* ---------------------------------------------------------------------------------------- */
    obstacles_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", qos,
        std::bind(&MotionPlannerNode::obstaclesCallback, this, std::placeholders::_1));

    borders_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "/borders", qos,
        std::bind(&MotionPlannerNode::bordersCallback, this, std::placeholders::_1));

    init_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino/amcl_pose", qos,
        std::bind(&MotionPlannerNode::startCallback, this, std::placeholders::_1));

    victims_timeout_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/victims_timeout", qos,
        std::bind(&MotionPlannerNode::victimsTimeoutCallback, this, std::placeholders::_1));

    /* ---------------------------------------------------------------------------------------- */
    /*                                      service                                             */
    /* ---------------------------------------------------------------------------------------- */
    service_ = this->create_service<motion_planner_msgs::srv::ValidatePath>(
        "validate_path",
        [this](
            const std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Request> request,
            std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Response> response)
        {
            validatePathCallback(request, response);
        });

    /* ---------------------------------------------------------------------------------------- */
    /*                                      publisher                                           */
    /* ---------------------------------------------------------------------------------------- */
    path_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("dubins_plan", qos);

    RCLCPP_INFO(this->get_logger(), "MotionPlannerNode is ready.");
}

/**
 * @brief Creates a QoS (Quality of Service) profile with transient local durability.
 *
 * This function generates a QoS profile with the specified depth, reliable reliability,
 * and transient local durability. Transient local durability ensures that messages
 * are stored by the middleware and delivered to late-joining subscribers.
 *
 * @param depth The depth of the QoS history (default is 10). This determines the number
 *              of messages to store in the history.
 * @return rclcpp::QoS A QoS profile configured with the specified settings.
 */
rclcpp::QoS MotionPlannerNode::get_transient_qos(size_t depth)
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
 * @brief Callback function to process received obstacle data.
 *
 * This function is triggered when a new obstacle array message is received. It clears the current
 * list of obstacles and populates it with the new data from the message. Each obstacle is
 * represented by its radius and a vector of points defining its polygon. Once the obstacles are
 * updated, a flag is set to indicate that obstacles have been received, and an attempt to generate
 * a motion plan is initiated.
 *
 * @param msg Shared pointer to the received ObstacleArrayMsg containing the obstacle data.
 */
void MotionPlannerNode::obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    obstacles_.clear();
    for (const auto &obs_msg : msg->obstacles)
    {
        obstacles_.emplace_back(obs_msg.radius,
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
 * This function is triggered when a new `PolygonStamped` message is received.
 * It processes the polygon points from the message, clears the existing borders,
 * and updates the internal borders list with the new points. Once the borders
 * are updated, it sets the `borders_received_` flag to true and logs the number
 * of points received. Finally, it attempts to generate a motion plan.
 *
 * @param msg A shared pointer to the received `geometry_msgs::msg::PolygonStamped` message.
 */
void MotionPlannerNode::bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
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
 * @brief Callback function to handle the reception of the start pose.
 *
 * This function is triggered when a start pose message is received. It checks
 * if a start pose has already been received. If so, it logs a warning and
 * ignores the new message. Otherwise, it processes the received start pose,
 * extracts the yaw angle, and logs the received pose information. Finally, it
 * attempts to generate a path based on the received start pose.
 *
 * @param msg A shared pointer to the received PoseWithCovarianceStamped message
 *            containing the start pose information.
 */
void MotionPlannerNode::startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (start_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Start pose already received, ignoring new message.");
        return; // Ignore if start pose is already set
    }
    th_start = tf2::getYaw(msg->pose.pose.orientation);
    start_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received start pose: (%.2f, %.2f) with yaw %.2f",
                msg->pose.pose.position.x, msg->pose.pose.position.y, th_start);
    attemptGenerate(); // Start path generation attempt
}

/**
 * @brief Callback function for handling the victims timeout message.
 *
 * This function is triggered when a new message is received on the victims timeout topic.
 * It sets the victims timeout value if it has not already been received. If a timeout
 * value has already been set, the new message is ignored, and a warning is logged.
 *
 * @param msg A shared pointer to the received message containing the timeout value.
 *            The timeout value is expected to be an integer.
 *
 * The function performs the following actions:
 * - If the timeout value has already been received, logs a warning and ignores the new message.
 * - If the timeout value is not set, assigns the received value to `victimsTimeout_`.
 *   If the received value is less than or equal to 0, a default value of 40 is used.
 * - Sets the `victimsTimeoutReceived_` flag to true.
 * - Logs the received timeout value.
 * - Calls the `attemptGenerate()` function to proceed with further processing.
 */
void MotionPlannerNode::victimsTimeoutCallback(const std_msgs::msg::Int32::SharedPtr msg)
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

/**
 * @brief Callback function to validate a given path using Dubins path planning.
 *
 * This function processes a path received in the request, validates it, and generates
 * a response indicating whether the path is valid. It also identifies failed and slow
 * segments in the path, if any, and logs relevant information.
 *
 * @param request Shared pointer to the service request containing the path to validate.
 * @param response Shared pointer to the service response to populate with validation results.
 *
 * The function performs the following steps:
 * - Clears and converts the input path to an internal representation.
 * - Attempts to generate a Dubins path based on the input path.
 * - Checks if the input path is empty or if the Dubins path generation failed.
 * - If the Dubins path is empty, identifies failed and slow segments, logs them, and
 *   adds them to the response.
 * - If the Dubins path is successfully generated, validates it and publishes the path.
 * - Logs the time taken for the validation process and the final validation result.
 */
void MotionPlannerNode::validatePathCallback(
    const std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Request> request,
    std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Response> response)
{
    pathFromMsg.clear();
    pathFromMsg = convertPoseArrayToPoints(request->path);
    path_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received path with %zu points.", pathFromMsg.size());
    dubinsPath.clear();

    // to start the timer for graph generation
    rclcpp::Time start_time = this->now();

    attemptGenerate();

    if (request->path.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty path, cannot validate.");
        response->valid = false;
        path_received_ = false;
    }
    else
    {
        if (dubinsPath.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Dubins path is empty, cannot validate.");
            response->valid = false;
            data_generated_ = false;
            path_received_ = false;

            failedSegments_ = getFailedSegments();
            if (!failedSegments_.empty())
            {

                for (const auto &segment : failedSegments_)
                {
                    const Point &p1 = segment.first;
                    const Point &p2 = segment.second;

                    response->failed_segments.push_back(convertToSegmentMsg(p1, p2));

                    RCLCPP_WARN(this->get_logger(), "Failed segment: Start (%.5f, %.5f), End (%.5f, %.5f)",
                                p1.getX(), p1.getY(), p2.getX(), p2.getY());
                }
            }

            slowSegments_ = getSlowSegments();
            if (!slowSegments_.empty())
            {

                for (const auto &segment : slowSegments_)
                {
                    const Point &p1 = segment.first;
                    const Point &p2 = segment.second;

                    response->slow_segments.push_back(convertToSegmentMsg(p1, p2));

                    RCLCPP_WARN(this->get_logger(), "Slow segment: Start (%.5f, %.5f), End (%.5f, %.5f)",
                                p1.getX(), p1.getY(), p2.getX(), p2.getY());
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Validating Dubins path with %zu arcs.", dubinsPath.size());
            response->valid = true;
            data_generated_ = true;
            publishDubinsPath();
        }
    }

    // to log the time taken for graph generation
    rclcpp::Time end_time = this->now();
    RCLCPP_INFO(get_logger(), "MOTIONPLANNERNODE: Service was working %f seconds.", (end_time - start_time).seconds());

    RCLCPP_INFO(this->get_logger(), "Path validation result: %s", response->valid ? "valid" : "invalid");
}

/* ------------------------------------------------------------------------------------------ */
/*                               attempt to generate                                          */
/* ------------------------------------------------------------------------------------------ */
/**
 * @brief Attempts to generate a Dubins path if all required data has been received.
 * 
 * This function checks if all necessary data (path, obstacles, borders, start position, 
 * and victim timeout) has been received. If so, it generates a Dubins path using the 
 * multiPointMarvkovDubinsPlan function. If not all data is available, it logs the 
 * missing data and waits for it to be received.
 * 
 * The generated path is stored in the `dubinsPath` variable, and a flag `data_generated_` 
 * is set to true to indicate that the path has been successfully generated.
 */
void MotionPlannerNode::attemptGenerate()
{
    if (path_received_ && obstacles_received_ && borders_received_ && start_received_ && victimsTimeoutReceived_ && !data_generated_)
    {
        RCLCPP_INFO(get_logger(), "All required data received; generating Dubins...");

        // print th
        RCLCPP_INFO(get_logger(), "Start angle: %.2f, Goal angle: %.2f", th_start, th_goal_);
        dubinsPath = multiPointMarvkovDubinsPlan(pathFromMsg, th_start, th_goal_, 16, 16, obstacles_, borders_, victimsTimeout_);

        data_generated_ = true;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Waiting for all data to be received: %s, %s, %s, %s, %s, %s",
                    path_received_ ? "yes" : "no",
                    obstacles_received_ ? "yes" : "no",
                    borders_received_ ? "yes" : "no",
                    start_received_ ? "yes" : "no",
                    victimsTimeoutReceived_ ? "yes" : "no",
                    data_generated_ ? "yes" : "no"
                );
    }
}

/* ------------------------------------------------------------------------------------------ */
/*                               convert PoseArray to Points                                  */
/* ------------------------------------------------------------------------------------------ */
/**
 * @brief Converts a PoseArray message into a vector of Points.
 * 
 * This function iterates through the poses in the given PoseArray message
 * and extracts their x and y positions to create a vector of Point objects.
 * Additionally, it calculates the yaw angle (th_goal_) from the orientation
 * of the last pose in the array. If the PoseArray is empty, th_goal_ is set
 * to 0.0 as a fallback.
 * 
 * @param pose_array_msg The PoseArray message containing the poses to be converted.
 * @return std::vector<Point> A vector of Points representing the x and y positions
 *         of the poses in the PoseArray.
 */
std::vector<Point> MotionPlannerNode::convertPoseArrayToPoints(const geometry_msgs::msg::PoseArray &pose_array_msg)
{
    std::vector<Point> points;
    for (const auto &pose : pose_array_msg.poses)
    {
        points.push_back(Point((double)(pose.position.x), (double)(pose.position.y)));
    }

    if (!pose_array_msg.poses.empty())
    {
        th_goal_ = tf2::getYaw(pose_array_msg.poses.back().orientation); // Yaw from last pose
    }
    else
    {
        th_goal_ = 0.0; // fallback if empty
    }
    return points;
}

/**
 * @brief Converts two points into a Segment message.
 * 
 * This function takes two points, `p1` and `p2`, and constructs a 
 * `motion_planner_msgs::msg::Segment` message. The start of the segment 
 * is set to the coordinates of `p1`, and the end of the segment is set 
 * to the coordinates of `p2`. The z-coordinate for both start and end 
 * points is set to 0.0.
 * 
 * @param p1 The starting point of the segment.
 * @param p2 The ending point of the segment.
 * @return A `motion_planner_msgs::msg::Segment` message representing 
 *         the segment from `p1` to `p2`.
 */
motion_planner_msgs::msg::Segment MotionPlannerNode::convertToSegmentMsg(const Point &p1, const Point &p2)
{
    motion_planner_msgs::msg::Segment segment;

    segment.start.x = p1.getX();
    segment.start.y = p1.getY();
    segment.start.z = 0.0;

    segment.end.x = p2.getX();
    segment.end.y = p2.getY();
    segment.end.z = 0.0;

    return segment;
}

/* ------------------------------------------------------------------------------------------ */
/*                               publish Dubins Path                                          */
/* ------------------------------------------------------------------------------------------ */
/**
 * @brief Publishes the Dubins path as a PoseArray message.
 * 
 * This method iterates through the Dubins path, converts each point's position
 * and heading (yaw) into a geometry_msgs::msg::Pose, and adds it to a PoseArray
 * message. The PoseArray message is then published to the configured topic.
 * 
 * The header of the PoseArray message is set with the frame ID "map" and the
 * current timestamp. The heading (yaw) of each point is converted to a quaternion
 * for the pose's orientation.
 * 
 * @note The Dubins path is assumed to be stored in the member variable `dubinsPath`,
 *       where each element contains the fields `x`, `y`, and `th` (heading in radians).
 * 
 * @details
 * - The `geometry_msgs::msg::PoseArray` message is used to represent the path.
 * - The `tf2::Quaternion` class is used to convert yaw to quaternion.
 * - The method logs the number of poses published using the RCLCPP_INFO macro.
 * 
 * @throws None
 */
void MotionPlannerNode::publishDubinsPath()
{
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "map";
    pose_array_msg.header.stamp = this->get_clock()->now();

    for (const auto &arc : dubinsPath)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = arc.x;
        pose.position.y = arc.y;
        pose.position.z = 0.0;

        // Convert heading (yaw) to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, arc.th);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        pose_array_msg.poses.push_back(pose);
    }

    path_publisher_->publish(pose_array_msg);
    RCLCPP_INFO(this->get_logger(), "Published Dubins path with %zu poses.", dubinsPath.size());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
