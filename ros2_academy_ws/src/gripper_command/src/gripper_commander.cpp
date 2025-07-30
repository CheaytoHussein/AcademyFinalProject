#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

const std::vector<std::string> GRIPPER_JOINTS = {
    "robotiq_85_left_knuckle_joint",
    "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint",
    "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint",
    "robotiq_85_right_finger_tip_joint"
};

// Adjust these values as needed for your gripper!
const std::vector<double> OPEN_POSITIONS  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const std::vector<double> CLOSE_POSITIONS = {0.8, -0.8, 0.8, -0.8, -0.8, 0.8};

void send_gripper_command(
    const std::shared_ptr<rclcpp::Node>& node,
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& pub,
    const std::vector<double>& positions
) {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = GRIPPER_JOINTS;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration(2, 0);  // 2 seconds
    traj.points.push_back(point);

    RCLCPP_INFO(node->get_logger(), "Sending gripper command:");
    for (auto v : positions) std::cout << v << " ";
    std::cout << std::endl;

    // Publish several times to ensure it's received
    for (int i = 0; i < 5; ++i) {
        pub->publish(traj);
        rclcpp::spin_some(node);
        rclcpp::sleep_for(100ms);
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gripper_commander_cpp");

    // Create publisher with reliable QoS
    auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/gripper_trajectory_controller/joint_trajectory",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    std::string mode = "open";
    if (argc > 1) mode = argv[1];

    if (mode == "open") {
        send_gripper_command(node, pub, OPEN_POSITIONS);
    } else if (mode == "close") {
        send_gripper_command(node, pub, CLOSE_POSITIONS);
    } else {
        std::cout << "Usage: gripper_commander_cpp [open|close]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Keep spinning to allow commands to be processed
    rclcpp::sleep_for(2s);

    rclcpp::shutdown();
    return 0;
}
