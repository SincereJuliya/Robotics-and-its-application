#include <rclcpp/rclcpp.hpp>
#include "motion_planner_msgs/srv/validate_path.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "../include/robotPlanning/point.hpp"
#include "../include/robotPlanning/multiPointMarkovDubins.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "../include/robotPlanning/obstacles.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode() : Node("motionPlannerNode") 
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

        /* ---------------------------------------------------------------------------------------- */
        /*                                      service                                             */
        /* ---------------------------------------------------------------------------------------- */
        service_ = this->create_service<motion_planner_msgs::srv::ValidatePath>(
            "validate_path", 
            [this](
                   const std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Request> request,
                   std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Response> response) {
                validatePathCallback(request, response);
            }
        );

        /* ---------------------------------------------------------------------------------------- */
        /*                                      publisher                                           */
        /* ---------------------------------------------------------------------------------------- */
        path_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("dubins_plan", qos);

        RCLCPP_INFO(this->get_logger(), "MotionPlannerNode is ready.");
    }

private:
    std::vector<Obstacle> obstacles_;
    std::vector<Point> borders_;
    std::vector<Point> pathFromMsg;
    std::vector<arcVar> dubinsPath;

    bool data_generated_ = false;
    bool path_received_ = false;
    bool obstacles_received_ = false;
    bool borders_received_ = false;

    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr borders_sub_;
    rclcpp::Service<motion_planner_msgs::srv::ValidatePath>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_publisher_;

    rclcpp::QoS get_transient_qos(size_t depth = 10) {
        rclcpp::QoS qos(depth);
        qos.reliable();
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        return qos;
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                                      callbacks                                             */
    /* ------------------------------------------------------------------------------------------ */
    void obstaclesCallback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg) {
        obstacles_.clear();
        for (const auto &obs_msg : msg->obstacles) {
            obstacles_.emplace_back(obs_msg.radius,
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
        borders_.clear();
        for (const auto &pt : msg->polygon.points) {
            borders_.emplace_back(pt.x, pt.y);
        }
        borders_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received borders with %zu points.", borders_.size());
        attemptGenerate();
    }

    void validatePathCallback(
                               const std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Request> request,
                               std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Response> response) 
    {
        pathFromMsg.clear();
        pathFromMsg = convertPoseArrayToPoints(request->path);
        path_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received path with %zu points.", pathFromMsg.size());
        dubinsPath.clear();
        
        attemptGenerate();

        if (request->path.poses.empty()) 
        {
            RCLCPP_WARN(this->get_logger(), "Received empty path, cannot validate.");
            response->valid = false;
            path_received_ = false;

        } 
        else 
        {
            if(dubinsPath.empty()) 
            {
                RCLCPP_WARN(this->get_logger(), "Dubins path is empty, cannot validate.");
                response->valid = false;
                data_generated_ = false;
                path_received_ = false;

            } else{
                RCLCPP_INFO(this->get_logger(), "Validating Dubins path with %zu arcs.", dubinsPath.size());
                response->valid = true;
                data_generated_ = true;
                publishDubinsPath();

            }

        }

        RCLCPP_INFO(this->get_logger(), "Path validation result: %s", response->valid ? "valid" : "invalid");

    }

    /* ------------------------------------------------------------------------------------------ */
    /*                               attempt to generate                                          */
    /* ------------------------------------------------------------------------------------------ */
    void attemptGenerate() 
    {
        if ( path_received_ && obstacles_received_ && borders_received_ && !data_generated_ ) 
        {
            RCLCPP_INFO(get_logger(), "All required data received; generating Dubins...");
            
            double th0 = 0.0;
            double thf = 0.0;

            if (pathFromMsg.size() >= 2) {
                // Initial heading from point 0 to point 1
                double dx0 = pathFromMsg[1].getX() - pathFromMsg[0].getX();
                double dy0 = pathFromMsg[1].getY() - pathFromMsg[0].getY();
                th0 = std::atan2(dy0, dx0);

                // Final heading from second-to-last to last point
                size_t n = pathFromMsg.size();
                double dxf = pathFromMsg[n-1].getX() - pathFromMsg[n-2].getX();
                double dyf = pathFromMsg[n-1].getY() - pathFromMsg[n-2].getY();
                thf = std::atan2(dyf, dxf);
            }

            dubinsPath = multiPointMarvkovDubinsPlan(pathFromMsg, th0, thf, 8, 16, obstacles_, borders_);
            data_generated_ = true;
        }
        else 
        {
            RCLCPP_INFO(get_logger(), "Waiting for all data to be received: "
                "path: %s, obstacles: %s",
                path_received_ ? "yes" : "no",
                obstacles_received_ ? "yes" : "no",
                borders_received_ ? "yes" : "no");
        }
    }

    /* ------------------------------------------------------------------------------------------ */
    /*                               convert PoseArray to Points                                  */
    /* ------------------------------------------------------------------------------------------ */
    // Method to convert PoseArray to a vector of Points
    std::vector<Point> convertPoseArrayToPoints(const geometry_msgs::msg::PoseArray& pose_array_msg) {
        std::vector<Point> points;
        for (const auto& pose : pose_array_msg.poses) {
            points.push_back(Point((double)(pose.position.x), (double)(pose.position.y)));
        }
        return points;

    }

    /* ------------------------------------------------------------------------------------------ */
    /*                               publish Dubins Path                                          */
    /* ------------------------------------------------------------------------------------------ */
    void publishDubinsPath() 
    {
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.frame_id = "map";
        pose_array_msg.header.stamp = this->get_clock()->now();

        for (const auto& arc : dubinsPath) 
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = arc.x;
            pose.position.y = arc.y;
            pose.position.z = 0.0;

            // Convert heading (yaw) to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, arc.th);     // Roll=0, Pitch=0, Yaw=theta
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            pose_array_msg.poses.push_back(pose);
        }

        path_publisher_->publish(pose_array_msg);
        RCLCPP_INFO(this->get_logger(), "Published Dubins path with %zu poses.", dubinsPath.size());

    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
