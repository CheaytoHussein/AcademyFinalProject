#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <thread>
#include <limits>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "academy_robot_approach_screw/action/approach_screw.hpp"

using namespace std::chrono_literals;
using ApproachScrew = academy_robot_approach_screw::action::ApproachScrew;

class ApproachScrewServer : public rclcpp::Node {
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ApproachScrew>;

  ApproachScrewServer() : Node("approach_screw_server")
  {
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    window_deg_ = this->declare_parameter<double>("window_deg", 20.0); // +/- 10 deg by default
    default_stop_distance_ = this->declare_parameter<double>("stop_distance", 0.35);
    default_max_speed_ = this->declare_parameter<double>("max_forward_speed", 0.15);
    kp_ = this->declare_parameter<double>("speed_kp", 0.8);  // v = clamp(kp*remaining, 0, max)

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ApproachScrewServer::onScan, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    action_server_ = rclcpp_action::create_server<ApproachScrew>(
      this,
      "approach_screw",
      std::bind(&ApproachScrewServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ApproachScrewServer::handleCancel, this, std::placeholders::_1),
      std::bind(&ApproachScrewServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ApproachScrew action server ready. Listening on action '/approach_screw'.");
  }

private:
  static double wrapAngle(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    last_scan_ = *scan;
    last_scan_time_ = this->now();
  }

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const ApproachScrew::Goal> goal)
  {
    (void)goal;
    RCLCPP_INFO(get_logger(), "Received goal: stop=%.3f m, vmax=%.2f m/s, timeout=%.1f s",
                goal->stop_distance, goal->max_forward_speed, goal->timeout_sec);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Cancel request received.");
    stopRobot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> gh)
  {
    // Run execution in separate thread
    std::thread([this, gh]() { this->execute(gh); }).detach();
  }

  std::optional<double> minRangeAhead(const sensor_msgs::msg::LaserScan& s) const {
    if (s.ranges.empty()) return std::nullopt;
    const double half = window_deg_ * M_PI / 180.0 / 2.0;
    const double ang_min = wrapAngle(-half);
    const double ang_max = wrapAngle(+half);

    double angle = s.angle_min;
    const double inc = s.angle_increment;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i=0; i<s.ranges.size(); ++i, angle += inc) {
      double a = wrapAngle(angle);
      bool in_window = (ang_min <= ang_max) ? (a >= ang_min && a <= ang_max)
                                            : (a >= ang_min || a <= ang_max);
      if (!in_window) continue;
      const float r = s.ranges[i];
      if (std::isfinite(r)) best = std::min(best, static_cast<double>(r));
    }
    if (!std::isfinite(best)) return std::nullopt;
    return best;
  }

  void stopRobot() {
    geometry_msgs::msg::Twist z;
    cmd_pub_->publish(z);
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    auto goal = gh->get_goal();
    const double stop_distance = (goal->stop_distance > 0.0) ? goal->stop_distance : default_stop_distance_;
    const double vmax = (goal->max_forward_speed > 0.0) ? goal->max_forward_speed : default_max_speed_;
    const double timeout = goal->timeout_sec;

    rclcpp::Time start = this->now();

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        stopRobot();
        auto res = std::make_shared<ApproachScrew::Result>();
        res->success = false;
        res->final_distance = 0.0f;
        res->message = "Canceled";
        gh->canceled(res);
        return;
      }

      // Ensure we have a recent scan
      if (!last_scan_time_ || (this->now() - *last_scan_time_).seconds() > 0.5) {
        // publish zero just in case
        stopRobot();
        if (timeout > 0.0 && (this->now() - start).seconds() > timeout) {
          auto res = std::make_shared<ApproachScrew::Result>();
          res->success = false;
          res->final_distance = 0.0f;
          res->message = "Timeout waiting for LaserScan";
          gh->abort(res);
          return;
        }
        rate.sleep();
        continue;
      }

      auto mr = minRangeAhead(last_scan_);
      if (!mr.has_value()) {
        stopRobot();
        if (timeout > 0.0 && (this->now() - start).seconds() > timeout) {
          auto res = std::make_shared<ApproachScrew::Result>();
          res->success = false;
          res->final_distance = 0.0f;
          res->message = "No target detected ahead";
          gh->abort(res);
          return;
        }
        rate.sleep();
        continue;
      }

      const double remaining = *mr - stop_distance;

      ApproachScrew::Feedback fb;
      fb.remaining_distance = static_cast<float>(remaining);
      gh->publish_feedback(std::make_shared<ApproachScrew::Feedback>(fb));

      if (remaining <= 0.0) {
        stopRobot();
        auto res = std::make_shared<ApproachScrew::Result>();
        res->success = true;
        res->final_distance = static_cast<float>(*mr);
        res->message = "Reached stop distance.";
        gh->succeed(res);
        return;
      }

      double v = std::min(vmax, std::max(0.0, kp_ * remaining));
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = v;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);

      if (timeout > 0.0 && (this->now() - start).seconds() > timeout) {
        stopRobot();
        auto res = std::make_shared<ApproachScrew::Result>();
        res->success = false;
        res->final_distance = static_cast<float>(*mr);
        res->message = "Timeout while approaching";
        gh->abort(res);
        return;
      }

      rate.sleep();
    }

    stopRobot();
  }

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  double window_deg_;
  double default_stop_distance_;
  double default_max_speed_;
  double kp_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp_action::Server<ApproachScrew>::SharedPtr action_server_;

  std::optional<rclcpp::Time> last_scan_time_;
  sensor_msgs::msg::LaserScan last_scan_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachScrewServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
