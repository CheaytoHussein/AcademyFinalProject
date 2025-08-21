#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "academy_robot_approach_screw/action/approach_screw.hpp"
#include "academy_robot_pose_interfaces/srv/get_screw_pose.hpp"

// MoveIt2
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

class PickPlaceOrchestrator : public rclcpp::Node {
public:
  using ApproachScrew = academy_robot_approach_screw::action::ApproachScrew;
  using GetScrewPose = academy_robot_pose_interfaces::srv::GetScrewPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ApproachScrew>;

  PickPlaceOrchestrator() : Node("pick_place_orchestrator")
  {
    // Parameters
    stop_distance_ = this->declare_parameter<double>("stop_distance", 0.35);
    forward_speed_ = this->declare_parameter<double>("max_forward_speed", 0.15);
    timeout_sec_ = this->declare_parameter<double>("approach_timeout", 30.0);

    place_frame_ = this->declare_parameter<std::string>("place_frame", "base_link");
    place_pose_x_ = this->declare_parameter<double>("place_x", 0.4);
    place_pose_y_ = this->declare_parameter<double>("place_y", -0.2);
    place_pose_z_ = this->declare_parameter<double>("place_z", 0.3);

    approach_client_ = rclcpp_action::create_client<ApproachScrew>(shared_from_this(), "approach_screw");
    pose_client_ = this->create_client<GetScrewPose>("get_screw_pose");

    // MoveIt setup
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_arm");
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "hand");

    RCLCPP_INFO(get_logger(), "Pick&Place orchestrator ready. Call 'ros2 run ... pick_place_orchestrator'");
  }

  void runOnce() {
    // 1) Approach
    if (!approach_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(), "Approach action unavailable.");
      return;
    }
    ApproachScrew::Goal goal;
    goal.stop_distance = stop_distance_;
    goal.max_forward_speed = forward_speed_;
    goal.timeout_sec = timeout_sec_;

    auto send_goal_options = rclcpp_action::Client<ApproachScrew>::SendGoalOptions();
    send_goal_options.feedback_callback = [this](GoalHandle::SharedPtr, const std::shared_ptr<const ApproachScrew::Feedback> fb){
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "Remaining distance: %.3f m", fb->remaining_distance);
    };
    auto future_goal_handle = approach_client_->async_send_goal(goal, send_goal_options);
    if (future_goal_handle.wait_for(60s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }
    auto gh = future_goal_handle.get();
    if (!gh) {
      RCLCPP_ERROR(get_logger(), "Goal rejected");
      return;
    }
    auto result_future = approach_client_->async_get_result(gh);
    if (result_future.wait_for(120s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Approach action result timeout");
      return;
    }
    auto res = result_future.get();
    if (res.code != rclcpp_action::ResultCode::SUCCEEDED || !res.result->success) {
      RCLCPP_ERROR(get_logger(), "Approach failed: %s", res.result ? res.result->message.c_str() : "no result");
      return;
    }
    RCLCPP_INFO(get_logger(), "Approach success. Final distance: %.3f m", res.result->final_distance);

    // 2) Get screw pose
    if (!pose_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "GetScrewPose service unavailable");
      return;
    }
    auto req = std::make_shared<GetScrewPose::Request>();
    auto fut = pose_client_->async_send_request(req);
    if (fut.wait_for(10s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "GetScrewPose timeout");
      return;
    }
    auto pose_res = fut.get();
    if (!pose_res->success) {
      RCLCPP_ERROR(get_logger(), "Pose estimation failed: %s", pose_res->message.c_str());
      return;
    }
    geometry_msgs::msg::PoseStamped target = pose_res->pose;
    RCLCPP_INFO(get_logger(), "Got screw pose in frame %s", target.header.frame_id.c_str());

    // 3) Simple pick and place motions (adjust to your MoveIt config/group names and grasp approach)
    // Open gripper
    (void)gripper_group_->setNamedTarget("open");
    gripper_group_->move();

    // Move above the screw (add an offset)
    geometry_msgs::msg::PoseStamped above = target;
    above.pose.position.z += 0.10; // 10 cm approach
    arm_group_->setPoseTarget(above);
    arm_group_->move();

    // Descend to grasp
    geometry_msgs::msg::PoseStamped grasp = target;
    grasp.pose.position.z += 0.01; // allow slight clearance
    arm_group_->setPoseTarget(grasp);
    arm_group_->move();

    // Close gripper
    (void)gripper_group_->setNamedTarget("close");
    gripper_group_->move();

    // Lift up
    arm_group_->setPoseTarget(above);
    arm_group_->move();

    // Place at a predefined location (in 'place_frame')
    geometry_msgs::msg::PoseStamped place;
    place.header.frame_id = place_frame_;
    place.header.stamp = this->get_clock()->now();
    place.pose.position.x = place_pose_x_;
    place.pose.position.y = place_pose_y_;
    place.pose.position.z = place_pose_z_;
    place.pose.orientation.w = 1.0;
    arm_group_->setPoseTarget(place);
    arm_group_->move();

    // Open gripper to release
    (void)gripper_group_->setNamedTarget("open");
    gripper_group_->move();

    RCLCPP_INFO(get_logger(), "Pick&Place sequence done (skeleton).");
  }

private:
  double stop_distance_, forward_speed_, timeout_sec_;
  std::string place_frame_;
  double place_pose_x_, place_pose_y_, place_pose_z_;

  rclcpp_action::Client<ApproachScrew>::SharedPtr approach_client_;
  rclcpp::Client<GetScrewPose>::SharedPtr pose_client_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceOrchestrator>();
  rclcpp::sleep_for(1s);
  node->runOnce();
  rclcpp::shutdown();
  return 0;
}
