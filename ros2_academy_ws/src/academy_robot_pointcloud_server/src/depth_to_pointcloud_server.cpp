#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "academy_robot_pointcloud_interfaces/srv/get_point_cloud.hpp"

using GetPointCloud = academy_robot_pointcloud_interfaces::srv::GetPointCloud;

class DepthToPointcloudServer : public rclcpp::Node {
public:
  DepthToPointcloudServer()
  : Node("depth_to_pointcloud_server")
  {
    // Parameter for the input cloud topic (your existing 'depth_to_pointcloud' publisher topic).
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/camera/depth/points");
    RCLCPP_INFO(get_logger(), "Subscribing to input topic: %s", input_topic_.c_str());

    // Cache the latest cloud.
    auto qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos,
        std::bind(&DepthToPointcloudServer::cloudCallback, this, std::placeholders::_1));

    // Service that returns the latest cached cloud.
    srv_ = this->create_service<GetPointCloud>(
        "get_latest_pointcloud",
        std::bind(&DepthToPointcloudServer::handleGetCloud, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Service 'get_latest_pointcloud' ready.");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    last_cloud_ = *msg;
  }

  void handleGetCloud(const std::shared_ptr<GetPointCloud::Request> /*req*/,
                      std::shared_ptr<GetPointCloud::Response> res)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (last_cloud_.header.stamp.sec == 0 && last_cloud_.header.stamp.nanosec == 0) {
      res->success = false;
      // Return an empty message if we have nothing yet.
      res->cloud = sensor_msgs::msg::PointCloud2();
      RCLCPP_WARN(get_logger(), "No cloud cached yet.");
    } else {
      res->success = true;
      res->cloud = last_cloud_;
    }
  }

  std::string input_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Service<GetPointCloud>::SharedPtr srv_;
  sensor_msgs::msg::PointCloud2 last_cloud_;
  std::mutex mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthToPointcloudServer>());
  rclcpp::shutdown();
  return 0;
}
