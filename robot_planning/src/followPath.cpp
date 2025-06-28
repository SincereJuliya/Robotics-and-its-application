#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "../include/robotPlanning/multiPointMarkovDubins.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace tf2;

class FollowPathActionClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit FollowPathActionClient(const rclcpp::NodeOptions &options)
      : Node("followPath", options)
  {
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    this->client_ptr_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino/follow_path");
    this->publisherRobot = this->create_publisher<nav_msgs::msg::Path>("shelfino/plan1", qos);
    this->planSubscriber = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "dubins_plan", qos, std::bind(&FollowPathActionClient::getPlan, this, std::placeholders::_1));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    if (plan.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Plan is empty, goal not sent.");
      return;
    }

    if (!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = FollowPath::Goal();
    goal_msg.controller_id = "FollowPath";
    goal_msg.goal_checker_id = "goal_checker";

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    for (size_t i = 0; i < plan.size(); i++)
    {
      geometry_msgs::msg::PoseStamped point;
      point.header = path_msg.header;
      point.pose.position.x = plan[i].x;
      point.pose.position.y = plan[i].y;
      point.pose.position.z = 0.0;

      // Correct quaternion from yaw (th)
      tf2::Quaternion q;
      q.setRPY(0, 0, plan[i].th);
      point.pose.orientation.x = q.x();
      point.pose.orientation.y = q.y();
      point.pose.orientation.z = q.z();
      point.pose.orientation.w = q.w();

      point.header.stamp = this->get_clock()->now();
      point.header.frame_id = "map";

      path_msg.poses.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Goal Parameters: Controller ID = %s, Goal Checker ID = %s",
                goal_msg.controller_id.c_str(), goal_msg.goal_checker_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Path size = %ld", path_msg.poses.size());

    publisherRobot->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Sending goal with %ld points", plan.size());
    goal_msg.path = path_msg;

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](const GoalHandle::SharedPtr &goal_handle)
    {
      this->goal_response_callback(goal_handle);
    };
    send_goal_options.feedback_callback =
        std::bind(&FollowPathActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&FollowPathActionClient::result_callback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Attempting to send goal to action server...");
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Goal sent, waiting for response...");
  }

private:
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<arcVar> plan;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherRobot;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr planSubscriber;
  std::vector<arcVar> msgToVec;

  void goal_response_callback(const GoalHandle::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "distance to goal: " << feedback->distance_to_goal << "\n";
    ss << "speed: " << feedback->speed << "\n";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandle::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    std::stringstream ss;
    ss << "Result received: " << result.result;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }

  std::vector<arcVar> convertPoseArrayToArcVars(const geometry_msgs::msg::PoseArray &pose_array_msg)
  {
    std::vector<arcVar> arc_vars;

    for (const auto &pose : pose_array_msg.poses)
    {
      double x = pose.position.x;
      double y = pose.position.y;

      tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      double th = yaw;

      arc_vars.push_back(arcVar{x, y, th});
    }

    return arc_vars;
  }

  void getPlan(const geometry_msgs::msg::PoseArray &pose_array_msg)
  {
    if (!plan.empty())
    {
      return;
    }

    msgToVec = convertPoseArrayToArcVars(pose_array_msg);
    for (size_t i = 0; i < msgToVec.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "Point %zu: x = %f, y = %f, th = %f", i, msgToVec[i].x, msgToVec[i].y, msgToVec[i].th);
      plan.push_back(msgToVec[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Plan size: %zu", plan.size());

    if (!plan.empty())
    {
      send_goal();
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<FollowPathActionClient>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
