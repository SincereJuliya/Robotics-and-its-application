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
    MotionPlannerNode() : Node("motionPlannerNode") {
        rclcpp::QoS qos(10);
        qos.reliable();
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        obstacles_sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", qos,
            std::bind(&MotionPlannerNode::obstaclesCallback, this, std::placeholders::_1));

        service_ = this->create_service<motion_planner_msgs::srv::ValidatePath>(
            "validate_path", 
            [this](
                   const std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Request> request,
                   std::shared_ptr<motion_planner_msgs::srv::ValidatePath::Response> response) {
                validatePathCallback(request, response);
            }
        );

        path_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("dubins_plan", qos);
        RCLCPP_INFO(this->get_logger(), "MotionPlanner service is ready.");
    }

private:
    std::vector<Obstacle> obstacles_;
    std::vector<Point> pathFromMsg;
    std::vector<arcVar> dubinsPath;

    bool data_generated_ = false;
    bool path_received_ = false;
    bool obstacles_received_ = false;

    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_sub_;
    rclcpp::Service<motion_planner_msgs::srv::ValidatePath>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_publisher_;

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

    void attemptGenerate() 
    {
        if ( path_received_ && obstacles_received_ && !data_generated_ ) {
            RCLCPP_INFO(get_logger(), "All required data received; generating Dubins...");
            
            dubinsPath = multiPointMarvkovDubinsPlan(pathFromMsg, 0,0, obstacles_);
            data_generated_ = true;
        }
        else {
            RCLCPP_INFO(get_logger(), "Waiting for all data to be received: "
                "path: %s, obstacles: %s",
                path_received_ ? "yes" : "no",
                obstacles_received_ ? "yes" : "no");
        }
    }

    // Method to convert PoseArray to a vector of Points
    std::vector<Point> convertPoseArrayToPoints(const geometry_msgs::msg::PoseArray& pose_array_msg) {
        std::vector<Point> points;
        for (const auto& pose : pose_array_msg.poses) {
            // Extract position and create Point objects
            points.push_back(Point((double)(pose.position.x), (double)(pose.position.y)));
        }

        return points;
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

        } else {
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

    void publishDubinsPath() {
        geometry_msgs::msg::PoseArray pose_array_msg;
        pose_array_msg.header.frame_id = "map";
        pose_array_msg.header.stamp = this->get_clock()->now();

        for (const auto& arc : dubinsPath) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = arc.x;
            pose.position.y = arc.y;
            pose.position.z = 0.0;

            // Convert heading (yaw) to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, arc.th);  // Roll=0, Pitch=0, Yaw=theta
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            pose_array_msg.poses.push_back(pose);
        }

        path_publisher_->publish(pose_array_msg);
        RCLCPP_INFO(this->get_logger(), "Published Dubins path with %zu poses.", dubinsPath.size());
    }


    // Method for FollowPathClass 
    /* // Method to convert PoseArray to a vector of arcVar objects
    std::vector<arcVar> convertPoseArrayToArcVars(const geometry_msgs::msg::PoseArray::SharedPtr& pose_array_msg) {
        std::vector<arcVar> arc_vars;

        for (const auto& pose : pose_array_msg->poses) {
            // Extract position (x, y, z) from the Pose
            double x = pose.position.x;
            double y = pose.position.y;
            double z = pose.position.z;

            // Extract orientation (quaternion)
            tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            
            // Convert quaternion to Euler angles (roll, pitch, yaw)
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // Use yaw (rotation around Z-axis) as heading (th)
            double th = yaw;

            // Create arcVar and add to vector
            arc_vars.push_back(arcVar(x, y, th));
        }

        return arc_vars;
    } */

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
