#include <rclcpp/rclcpp.hpp>
#include <graph_for_task_planner_msg/msg/graph.hpp>
#include <graph_for_task_planner_msg/msg/point.hpp>
#include <graph_for_task_planner_msg/msg/edge.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "geometry_msgs/msg/polygon_stamped.hpp"

#include <obstacles_msgs/msg/obstacle_array_msg.hpp> 
#include <obstacles_msgs/msg/obstacle_msg.hpp>  

#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/IPathPlanner.hpp"
#include "../include/robotPlanning/agreedy.hpp"
#include "../include/robotPlanning/victim.hpp"

#include "motion_planner_msgs/srv/validate_path.hpp"

/* ---------------------------------------------------------------------------------------- */
/*          node for task planning - for the A* algorithm to find a path in a graph         */
/* ---------------------------------------------------------------------------------------- */
class TaskPlannerNode : public rclcpp::Node {
public:
    TaskPlannerNode() : Node("taskPlannerNode") 
    {
        auto qos = get_transient_qos();

        /* ---------------------------------------------------------------------------------------- */
        /*                                        subscriptions                                     */
        /* ---------------------------------------------------------------------------------------- */
        gates_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gates", qos,
            std::bind(&TaskPlannerNode::gatesCallback, this, std::placeholders::_1));

        init_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/shelfino/amcl_pose", qos,
            std::bind(&TaskPlannerNode::startCallback, this, std::placeholders::_1));

        graph_subscriber_ = this->create_subscription<graph_for_task_planner_msg::msg::Graph>(
            "/generated_graph", qos,
            std::bind(&TaskPlannerNode::graphCallback, this, std::placeholders::_1));

        victims_subscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/victims", qos,
            std::bind(&TaskPlannerNode::victimsCallback, this, std::placeholders::_1));

        obstacles_subscriber_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", qos,
            std::bind(&TaskPlannerNode::obstaclesCallback, this, std::placeholders::_1));
        
        borders_subscriber_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/borders", qos,
            std::bind(&TaskPlannerNode::bordersCallback, this, std::placeholders::_1));
        
        /* ---------------------------------------------------------------------------------------- */
        /*          service to generate a path till its not validated by Motion Planner             */
        /* ---------------------------------------------------------------------------------------- */
        motion_planner_client_ = this->create_client<motion_planner_msgs::srv::ValidatePath>(
            "validate_path");

        RCLCPP_INFO(this->get_logger(), "TaskPlannerNode is ready.");
    }

private:
    std::unique_ptr<IPathPlanner> planner_;

    std::vector<Point> gates_;
    int attempt_count_ = 0;  // Track the number of attempts to generate a path
    Point start;
    std::vector<Victim> victims_;  // Store victim positions
    std::vector<Obstacle> obstacles_;  // Store obstacle positions
    std::vector<Point> borders_;

    bool start_received_ = false;
    bool gates_received_ = false;
    bool graph_received_ = false;
    bool victims_received_ = false;
    bool obstacles_received_ = false;
    bool borders_received_ = false;

    bool data_generated_ = false;
    bool path_validated_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gates_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_subscriber_;
    rclcpp::Subscription<graph_for_task_planner_msg::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr victims_subscriber_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_subscriber_;
    
    rclcpp::Client<motion_planner_msgs::srv::ValidatePath>::SharedPtr motion_planner_client_;

    Graph graph_;

    rclcpp::QoS get_transient_qos(size_t depth = 10) {
        rclcpp::QoS qos(depth);
        qos.reliable();
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        return qos;
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                                      callbacks                                             */
    /* ------------------------------------------------------------------------------------------ */
    void gatesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if(gates_received_) {
            RCLCPP_WARN(this->get_logger(), "Gates already received, ignoring new message.");
            return;  // Ignore if gates are already set
        }
        gates_.clear();
        for (const auto& pose : msg->poses) {
            gates_.emplace_back(pose.position.x, pose.position.y);
        }
        gates_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received %zu gates.", gates_.size());
        attemptGenerate();  // Start path generation attempt

    }

    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        if(start_received_) {
            RCLCPP_WARN(this->get_logger(), "Start pose already received, ignoring new message.");
            return;  // Ignore if start pose is already set
        }
        start = Point{msg->pose.pose.position.x, msg->pose.pose.position.y};
        start_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received start pose: (%.2f, %.2f)", start.getX(), start.getY());
        attemptGenerate();  // Start path generation attempt
    }

    void graphCallback(const graph_for_task_planner_msg::msg::Graph::SharedPtr msg) {
        if(graph_received_) {
            RCLCPP_WARN(this->get_logger(), "Graph already received, ignoring new message.");
            return;  // Ignore if graph is already set
        }

        graph_.clear();
        for (const auto& v : msg->vertices)
            graph_.addVertice(Point(v.x, v.y));

        for (const auto& e : msg->edges)
            graph_.addEdge(Point(e.start_point.x, e.start_point.y), Point(e.end_point.x, e.end_point.y));

        graph_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received graph with %zu vertices and %zu edges.",
                    msg->vertices.size(), msg->edges.size());
        attemptGenerate();  // Start path generation attempt
    }

    void victimsCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        if(victims_received_) {
            RCLCPP_WARN(this->get_logger(), "Victims already received, ignoring new message.");
            return;  // Ignore if victims are already set
        }

        victims_.clear();
        for (const auto& obstacle : msg->obstacles) {
            // Construct a Victim object using x, y, and radius
            if (!obstacle.polygon.points.empty()) {
                Victim v(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y, obstacle.radius);
                victims_.push_back(v);
            }
        }
        victims_received_ = !victims_.empty();
        RCLCPP_INFO(this->get_logger(), "Received %zu victims.", victims_.size());
        attemptGenerate();  // Start path generation attempt
    }

    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        if(obstacles_received_) {
            RCLCPP_WARN(this->get_logger(), "Obstacles already received, ignoring new message.");
            return;  // Ignore if obstacles are already set
        }

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
        RCLCPP_INFO(this->get_logger(), "Received %zu obstacles.", obstacles_.size());
        attemptGenerate();
    }

    void bordersCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        if(borders_received_) {
            RCLCPP_WARN(this->get_logger(), "Borders already received, ignoring new message.");
            return;  // Ignore if Borders are already set
        }

        borders_.clear();
        for (const auto &pt : msg->polygon.points) {
            borders_.emplace_back(pt.x, pt.y);
        }
        borders_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received borders with %zu points.", borders_.size());
        attemptGenerate();
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                attempt to generate the path if everything already come                     */
    /* ------------------------------------------------------------------------------------------ */
    void attemptGenerate() 
    {
        if (path_validated_) {
            RCLCPP_INFO(this->get_logger(), "Path already validated.");
            return;
        }

        if (attempt_count_ > 6) {
            RCLCPP_ERROR(this->get_logger(), "Exceeded max retry attempts.");
            return;
        }

        if (!gates_received_ || !start_received_ || !graph_received_ 
                || !victims_received_ || !obstacles_received_ || !borders_received_ || data_generated_) 
        {
            RCLCPP_INFO(get_logger(), "Waiting for all data to be received...");
            RCLCPP_INFO(get_logger(), "Start: %s, Gates: %s, Graph: %s, Victims: %s, Obstacles: %s,Data Generated: %s, Borders: %s",
                        start_received_ ? "yes" : "no",
                        gates_received_ ? "yes" : "no",
                        graph_received_ ? "yes" : "no",
                        victims_received_ ? "yes" : "no",
                        obstacles_received_ ? "yes" : "no",
                        borders_received_ ? "yes" : "no",
                        data_generated_ ? "yes" : "no"
            );
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generating path on attempt %d", attempt_count_);

        std::vector<Point> path = generatePath(attempt_count_);

        validatePath(path);  // Use the async version with callback
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                          generate the path using A* Greedy algorithm                       */
    /* ------------------------------------------------------------------------------------------ */
    std::vector<Point> generatePath(int attempt) 
    {
        if (start_received_ && gates_received_ && graph_received_ && victims_received_ && obstacles_received_ && borders_received_)  // Ensure all data is available
        { 
            Graph graph = graph_;  // Create a copy of the graph bc which gates depend on the attempt number, i dont want to add both

            Point start = this->start;  // Use the start point received from AMCL
            connectVisibleEdges(graph, start);
            RCLCPP_INFO(this->get_logger(), "Start point integrated into the graph.");

            Point goal = attempt <= 2 ? gates_.front() : gates_.back();
            connectVisibleEdges(graph, goal);
            RCLCPP_INFO(this->get_logger(), "Goal point integrated into the graph.");

            integrateVictimsIntoGraph(graph, victims_);
            RCLCPP_INFO(this->get_logger(), "Victims integrated into the graph.");
            

            RCLCPP_INFO(this->get_logger(), "Planning A* from (%.2f, %.2f) to (%.2f, %.2f)", start.getX(), start.getY(), goal.getX(), goal.getY());

            // Set time limit based on the attempt number
            int stx = attempt <= 2 ? 0 : 3;

            //
            // thats a topic
            double timeLimit = ( 60 * std::pow(0.8, attempt - stx) );

            AStarGreedy planner(graph, victims_, start, goal, timeLimit, obstacles_, borders_);
            RCLCPP_INFO(this->get_logger(), "A* Greedy planner initialized with time limit: %.2f seconds", timeLimit);

            double valueCollected = 0.0;
            std::vector<Point> bestPath = planner.run(valueCollected, attempt);

            std::cout << "Collected value: " << valueCollected << std::endl;

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
                borders_received_ ? "yes" : "no"
            );
            return {};  // Return an empty path if data is not ready
        }
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                          validate the path using Motion Planner service                    */
    /* ------------------------------------------------------------------------------------------ */
    void validatePath(const std::vector<Point>& path) 
    {
        if (!motion_planner_client_->wait_for_service(std::chrono::seconds(5))) 
        {
            RCLCPP_ERROR(this->get_logger(), "ValidatePath service unavailable.");
            return;
        }

        auto request = std::make_shared<motion_planner_msgs::srv::ValidatePath::Request>();
        request->path = convertToPathMessage(path);

        // Send the request asynchronously and handle the response in the callback
        auto future_result = motion_planner_client_->async_send_request(request,
            [this, path](rclcpp::Client<motion_planner_msgs::srv::ValidatePath>::SharedFuture result) 
            {
                if (result.get()->valid) 
                {
                    RCLCPP_INFO(this->get_logger(), "Generated path is valid.");
                    data_generated_ = true;
                    path_validated_ = true;

                } 
                else {
                    RCLCPP_WARN(this->get_logger(), "Generated path is invalid (attempt %d)", attempt_count_);
                    data_generated_ = false;
                    if (attempt_count_ < 6) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Retrying...");
                        attempt_count_ += 1;

                        attemptGenerate();  // triggers a new path generation

                    } 
                    else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to generate a valid path after 5 attempts.");

                    }

                }

            }

        );

    }

    /* ------------------------------------------------------------------------------------------ */
    /*                          integrate the point into the graph                                */
    /* ------------------------------------------------------------------------------------------ */
    bool collidesWithObstacleOrBorder(const Point& p1, const Point& p2) const 
    {
        double distance = std::hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY());

        // 3 samples per one unit of distance
        const double samplesPerUnit = 3.0; 
        int numSamples = std::max(1, static_cast<int>(distance * samplesPerUnit));

        for (int i = 0; i <= numSamples; ++i) {
            double t = static_cast<double>(i) / numSamples;
            Point interp(
                p1.getX() + t * (p2.getX() - p1.getX()),
                p1.getY() + t * (p2.getY() - p1.getY())
            );

            for (const auto& obs : obstacles_) {
                if (obs.isTooCloseToObstacle(interp, 0.6f) || obs.isInsideObstacle(interp)) {
                    return true;
                }
            }
        }

        return false; // No collision detected
    }

    void connectVisibleEdges(Graph& graph, const Point& newP) const 
    {
        std::vector<Point> victims_points;
        for (const auto& victim : victims_) {
            victims_points.emplace_back(victim.x, victim.y);
        }

        const auto& verts = graph.getVertices();
        for (const auto& v : verts) {
            if (v == newP) continue;

            if ( newP == start && ( std::find(victims_points.begin(), victims_points.end(), v) != victims_points.end() || std::find(gates_.begin(), gates_.end(), v) != gates_.end())) 
            {
                std::cout << "Skipping connection from start to victim point: " << v.toString() << std::endl;
                continue;
            }

            if ( std::find(victims_points.begin(), victims_points.end(), newP) != victims_points.end() 

                    && ( std::find(victims_points.begin(), victims_points.end(), v) != victims_points.end() 
                    || v == start
                    || std::find(gates_.begin(), gates_.end(), v) != gates_.end() )
                    
            )
            {
                std::cout << "Skipping connection from victim point to start: " << v.toString() << std::endl;
                continue;
            }

            if ( std::find(gates_.begin(), gates_.end(), newP) != gates_.end()
                    && ( std::find(victims_points.begin(), victims_points.end(), v) != victims_points.end() || v == start)
            )
            { 
                std::cout << "Skipping connection from gate to victim point: " << v.toString() << std::endl;
                continue;
            }


            if (!graph.edgeExists(newP, v) && !collidesWithObstacleOrBorder(newP, v)) {
                graph.addEdge(newP, v);
            }
        }
    }

    void integratePointIntoGraph(Graph& graph, Point& point) 
    {
        graph.addVertice(point);
        // Находим ближайшую вершину в графе
        Point nearest = graph.findNearestPoint(point);
        // Связываем с ближайшей точкой графа
        graph.addEdge(point, nearest);
        
    }

    void integrateVictimsIntoGraph(Graph& graph, const std::vector<Victim>& victims) 
    {
        for (const auto& victim : victims) {
            Point victimPoint(victim.x, victim.y);
            connectVisibleEdges(graph, victimPoint);
        }
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                          convert the path to a ROS message format                          */
    /* ------------------------------------------------------------------------------------------ */
    geometry_msgs::msg::PoseArray convertToPathMessage(const std::vector<Point>& path) {
        geometry_msgs::msg::PoseArray msg;
        for (const auto& p : path) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.getX();
            pose.position.y = p.getY();
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }
        return msg;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
