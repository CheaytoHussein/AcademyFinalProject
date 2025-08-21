#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "academy_robot_pose_interfaces/srv/get_screw_pose.hpp"

using GetScrewPose = academy_robot_pose_interfaces::srv::GetScrewPose;

class ScrewPoseService : public rclcpp::Node {
public:
  ScrewPoseService() : Node("screw_pose_service") {
    // Latched (transient local) subscriber to match your publisher QoS
    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable();
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_estimate", latched_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){
        last_pose_ = *msg;
        have_pose_ = true;
      });

    srv_ = this->create_service<GetScrewPose>(
      "get_screw_pose",
      [this](const GetScrewPose::Request::SharedPtr,
             GetScrewPose::Response::SharedPtr res){
        if (!have_pose_) {
          res->success = false;
          res->message = "No pose_estimate received yet";
        } else {
          res->success = true;
          res->pose = last_pose_;
          res->message = "OK";
        }
      });

    RCLCPP_INFO(get_logger(), "Service 'get_screw_pose' ready (wrapping /pose_estimate).");
  }
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Service<GetScrewPose>::SharedPtr srv_;
  geometry_msgs::msg::PoseStamped last_pose_;
  bool have_pose_ = false;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScrewPoseService>());
  rclcpp::shutdown();
  return 0;
}
