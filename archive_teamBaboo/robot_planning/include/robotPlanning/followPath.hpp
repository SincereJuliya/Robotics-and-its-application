#ifndef FOLLOW_PATH_ACTION_CLIENT_HPP
#define FOLLOW_PATH_ACTION_CLIENT_HPP

/**
 * @file FollowPathActionClient.hpp
 * @brief Declares the FollowPathActionClient class, which sends planned Dubins paths to the Nav2 action server.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../include/robotPlanning/multiPointMarkovDubins.hpp"

/**
 * @class FollowPathActionClient
 * @brief Action client for sending a Dubins path plan to the Nav2 FollowPath action server.
 */
class FollowPathActionClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  /**
   * @brief Construct a new Follow Path Action Client object.
   * @param options ROS node options.
   */
  explicit FollowPathActionClient(const rclcpp::NodeOptions &options);

  /**
   * @brief Sends the planned path as a goal to the FollowPath action server.
   */
  void send_goal();

private:
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<arcVar> plan;
  std::vector<arcVar> msgToVec;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisherRobot;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr planSubscriber;

  /**
   * @brief Called when the server responds to a goal request.
   * @param goal_handle Handle to the accepted or rejected goal.
   */
  void goal_response_callback(const GoalHandle::SharedPtr &goal_handle);

  /**
   * @brief Called when feedback is received from the server.
   * @param goal_handle Handle to the goal.
   * @param feedback Feedback message from the server.
   */
  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback);

  /**
   * @brief Called when the result of the goal is received.
   * @param result The result wrapper from the action server.
   */
  void result_callback(const GoalHandle::WrappedResult &result);

  /**
   * @brief Converts a PoseArray message to a vector of arcVar.
   * @param pose_array_msg PoseArray input from the planner.
   * @return std::vector<arcVar> Converted poses.
   */
  std::vector<arcVar> convertPoseArrayToArcVars(const geometry_msgs::msg::PoseArray &pose_array_msg);

  /**
   * @brief Callback function for receiving the planned Dubins path.
   * @param pose_array_msg The received PoseArray message.
   */
  void getPlan(const geometry_msgs::msg::PoseArray &pose_array_msg);
};

#endif // FOLLOW_PATH_ACTION_CLIENT_HPP
