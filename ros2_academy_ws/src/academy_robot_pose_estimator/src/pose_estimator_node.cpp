#include <chrono>
#include <memory>
#include <string>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h> // for loadPolygonFileSTL
#include <pcl/PolygonMesh.h>
#include <Eigen/Geometry>

#include "academy_robot_pointcloud_interfaces/srv/get_point_cloud.hpp"
#include "academy_robot_pose_estimator/registration.hpp"

using namespace std::chrono_literals;
using GetPointCloud = academy_robot_pointcloud_interfaces::srv::GetPointCloud;

class PoseEstimatorNode : public rclcpp::Node {
public:
  PoseEstimatorNode()
  : Node("pose_estimator_node")
  {
    // Parameters
    mesh_path_ = this->declare_parameter<std::string>(
      "mesh_path",
      "ros2_academy_ws/src/academy_robot_simulation/academy_robot_gazebo_ignition/meshes/socket_cap_screw.stl");
    service_name_ = this->declare_parameter<std::string>("service_name", "/get_latest_pointcloud");
    call_period_ms_ = this->declare_parameter<int>("call_period_ms", 1000);
    mesh_scale_ = this->declare_parameter<double>("mesh_scale", 1.0); // in case STL units differ
    params_.voxel_leaf = this->declare_parameter<double>("voxel_leaf", 0.005);
    params_.normal_radius = this->declare_parameter<double>("normal_radius", 0.01);
    params_.feature_radius = this->declare_parameter<double>("feature_radius", 0.025);

    // Latched publisher (transient_local) for pose estimate
    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local().reliable();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_estimate", latched_qos);

    // Service client
    client_ = this->create_client<GetPointCloud>(service_name_);

    // Load model mesh once
    if (!loadModel(mesh_path_)) {
      RCLCPP_FATAL(get_logger(), "Failed to load STL mesh: %s", mesh_path_.c_str());
      throw std::runtime_error("Failed to load mesh");
    }

    // Timer to trigger requests (non-blocking; response handled via async callback)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(call_period_ms_),
      std::bind(&PoseEstimatorNode::tick, this));

    RCLCPP_INFO(get_logger(), "PoseEstimatorNode ready. Requesting from service: %s", service_name_.c_str());
  }

private:
  bool loadModel(const std::string& path) {
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(path, mesh) <= 0) {
      RCLCPP_ERROR(get_logger(), "Could not load STL file: %s", path.c_str());
      return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr verts(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *verts);
    // Optional scaling
    if (std::abs(mesh_scale_ - 1.0) > 1e-6) {
      for (auto& p : verts->points) {
        p.x *= mesh_scale_;
        p.y *= mesh_scale_;
        p.z *= mesh_scale_;
      }
    }
    model_cloud_ = verts;
    RCLCPP_INFO(get_logger(), "Loaded model vertices: %zu", model_cloud_->size());
    return true;
  }

  void tick() {
    // Keep this very short/non-blocking
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                           "Service %s not available yet...", service_name_.c_str());
      return;
    }

    auto req = std::make_shared<GetPointCloud::Request>();

    // Async request; handle response in callback (no spin_until_future_complete)
    client_->async_send_request(
      req,
      std::bind(&PoseEstimatorNode::handleResponse, this, std::placeholders::_1));
  }

  void handleResponse(rclcpp::Client<GetPointCloud>::SharedFuture future) {
    auto res = future.get();

    if (!res->success) {
      RCLCPP_WARN(get_logger(), "Service returned no cloud.");
      return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(res->cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *scene);

    if (scene->empty()) {
      RCLCPP_WARN(get_logger(), "Received empty scene cloud; skipping.");
      return;
    }

    // Register model to scene
    Eigen::Matrix4f T;
    std::string debug;
    bool ok = registerModelToScene(model_cloud_, scene, T, params_, &debug);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Registration failed. %s", debug.c_str());
      return;
    }

    // Compose PoseStamped
    geometry_msgs::msg::PoseStamped pose;
    pose.header = res->cloud.header; // same frame as the scene cloud
    Eigen::Matrix3f R = T.block<3,3>(0,0);
    Eigen::Vector3f t = T.block<3,1>(0,3);
    Eigen::Quaternionf q(R);
    q.normalize();

    pose.pose.position.x = t.x();
    pose.pose.position.y = t.y();
    pose.pose.position.z = t.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    pose_pub_->publish(pose);
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "Published pose estimate (latched).");
  }

  // Members
  std::string mesh_path_;
  std::string service_name_;
  int call_period_ms_;
  double mesh_scale_;
  RegistrationParams params_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Client<GetPointCloud>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
