/**
 * @file FollowPathActionClient.cpp
 * @brief Implementation of the FollowPathActionClient class.
 *
 * This node subscribes to a Dubins‑style plan (as a PoseArray),
 * converts it to a nav_msgs/Path, and sends it as a goal to the
 * Nav2 *FollowPath* action server.  It also republishes the path
 * for visualisation 
 */

#include "FollowPathActionClient.hpp"
#include <sstream>

using namespace tf2;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Construct a new FollowPathActionClient.
 *
 * Creates:
 *  - an action client to `/shelfino/follow_path`
 *  - a publisher (`shelfino/plan1`) for visualising the path
 *  - a subscription (`dubins_plan`) that receives the Dubins plan
 *
 * @param options Standard ROS 2 node options forwarded to rclcpp::Node.
 */
FollowPathActionClient::FollowPathActionClient(const rclcpp::NodeOptions &options)
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

// ─────────────────────────────────────────────────────────────────────────────
// send_goal
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Converts the plan to a nav_msgs/Path and sends it to the action server.
 *
 * 1. Publishes the path to `shelfino/plan1` for RViz.
 * 2. Sends the same path as a goal to the Nav2 FollowPath server.
 * 3. Wires up goal‑response, feedback and result callbacks.
 *
 * If the plan is empty or the server is unavailable, the function exits early.
 */
void FollowPathActionClient::send_goal()
  {
    using namespace std::placeholders;
    // ── Safety checks ----------------------------------------------------
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
    // ── Build nav_msgs/Path ----------------------------------------------------
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

    // ── Send goal --------------------------------------------------------------
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


// ─────────────────────────────────────────────────────────────────────────────
// goal_response_callback
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Called once the server approves or rejects the goal.
 *
 * @param goal_handle Handle that is valid if and only if the goal was accepted.
 */
void FollowPathActionClient::goal_response_callback(const GoalHandle::SharedPtr &goal_handle)
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

// ─────────────────────────────────────────────────────────────────────────────
// feedback_callback
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Logs real‑time feedback from the controller.
 *
 * @param goal_handle     Unused.
 * @param feedback        Shared pointer to feedback message.
 */
void FollowPathActionClient::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  std::stringstream ss;
  ss << "distance to goal: " << feedback->distance_to_goal << "\n";
  ss << "speed: " << feedback->speed << "\n";
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// result_callback
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Handles the final result from the server.
 *
 * @param result Wrapped result containing status and user data.
 */
void FollowPathActionClient::result_callback(const GoalHandle::WrappedResult &result)
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


// ─────────────────────────────────────────────────────────────────────────────
// convertPoseArrayToArcVars
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Helper: converts a PoseArray into a vector of @c arcVar.
 *
 * @param pose_array_msg The incoming PoseArray.
 * @return std::vector<arcVar> Vector where each element holds x, y, yaw.
 */
std::vector<arcVar> FollowPathActionClient::convertPoseArrayToArcVars(const geometry_msgs::msg::PoseArray &pose_array_msg)
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

// ─────────────────────────────────────────────────────────────────────────────
// getPlan
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Callback for the `dubins_plan` subscription.
 *
 * Converts the incoming pose array, stores it in @c plan, then calls
 * #send_goal once (first message only).
 *
 * @param pose_array_msg The planner’s PoseArray message.
 */
void FollowPathActionClient::getPlan(const geometry_msgs::msg::PoseArray &pose_array_msg)
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

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Entry point when running the node .
 */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<FollowPathActionClient>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
