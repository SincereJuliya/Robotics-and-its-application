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

class FollowPathActionClient:public rclcpp::Node{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  explicit FollowPathActionClient(const rclcpp::NodeOptions & options):Node("followPath", options){
    this->client_ptr_ = rclcpp_action::create_client<FollowPath>(
      this,
      "/shelfino/follow_path");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FollowPathActionClient::send_goal, this));
  }

/*   void send_goal(){//general version
    using namespace std::placeholders;

    this->timer_->cancel();

    if(!this->client_ptr_->wait_for_action_server()){
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = FollowPath::Goal();
    goal_msg.controller_id = "followPath1";
    goal_msg.goal_checker_id = "followPath1";

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FollowPathActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FollowPathActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FollowPathActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  } */
 void send_goal() {
  using namespace std::placeholders;

  this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  auto goal_msg = FollowPath::Goal();
  goal_msg.controller_id = "FollowPath";
  goal_msg.goal_checker_id = "goal_checker";

  // Define the path with three points
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map"; 

  for(int i =0; i<10; i++){
    geometry_msgs::msg::PoseStamped point;
    point.header = path_msg.header;
    point.pose.position.x = (i*0.1);
    point.pose.position.y = (i*0.1);
    point.pose.orientation.w = 1.0;

    path_msg.poses.push_back(point);
  }


  goal_msg.path = path_msg;

  RCLCPP_INFO(this->get_logger(), "Sending goal with 3 points");

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&FollowPathActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&FollowPathActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&FollowPathActionClient::result_callback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}


private:
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle){
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback){
    std::stringstream ss;
    ss << "distance to goal: " << feedback->distance_to_goal << "\n";
    ss << "speed: " << feedback->speed << "\n";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandle::WrappedResult & result)
  {
    switch (result.code) {
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
}; 

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<FollowPathActionClient>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}