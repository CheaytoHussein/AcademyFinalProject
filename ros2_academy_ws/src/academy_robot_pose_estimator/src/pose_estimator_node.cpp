#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <random>
#include <algorithm>

#include <Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/vtk_lib_io.h>     // loadPolygonFileSTL

#include <academy_robot_pointcloud_interfaces/srv/get_point_cloud.hpp>

using GetPointCloud = academy_robot_pointcloud_interfaces::srv::GetPointCloud;

class PoseEstimatorNode : public rclcpp::Node {
public:
  PoseEstimatorNode()
  : rclcpp::Node("pose_estimator_node")
  {
    // Parameters
    service_name_  = this->declare_parameter<std::string>("service_name", "/get_latest_pointcloud");
    mesh_path_     = this->declare_parameter<std::string>("mesh_path", "");
    voxel_leaf_    = this->declare_parameter<double>("voxel_leaf", 0.015);
    normal_radius_ = this->declare_parameter<double>("normal_radius", 0.03);
    feature_radius_= this->declare_parameter<double>("feature_radius", 0.06);
    roi_min_z_  = this->declare_parameter<double>("roi_min_z", 0.05);  // meters
    roi_max_z_  = this->declare_parameter<double>("roi_max_z", 3.0);   // meters
    roi_half_xy_ = this->declare_parameter<double>("roi_half_xy", 1.0); // +/- meters in x,y
    remove_plane_ = this->declare_parameter<bool>("remove_plane", true);
    min_scene_pts_  = this->declare_parameter<int>("min_scene_pts", 2000);
    min_model_pts_  = this->declare_parameter<int>("min_model_pts", 5000);
    max_model_pts_  = this->declare_parameter<int>("max_model_pts", 80000);

    roi_min_z_   = this->declare_parameter<double>("roi_min_z", 0.05);
    roi_max_z_   = this->declare_parameter<double>("roi_max_z", 3.0);
    roi_half_xy_ = this->declare_parameter<double>("roi_half_xy", 1.2);
    remove_plane_= this->declare_parameter<bool>("remove_plane", false);

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_estimate", latched_qos);

    client_ = this->create_client<GetPointCloud>(service_name_);

    if (!loadModel(mesh_path_)) {
      RCLCPP_FATAL(get_logger(), "Failed to load mesh: %s", mesh_path_.c_str());
      throw std::runtime_error("Failed to load mesh");
    }
    RCLCPP_INFO(get_logger(), "Loaded model vertices: %zu", model_raw_->size());

    // 5 Hz tick
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                     std::bind(&PoseEstimatorNode::tick, this));

    RCLCPP_INFO(get_logger(), "PoseEstimatorNode ready. Requesting from service: %s",
                service_name_.c_str());
  }

private:
  // ===== model load & preprocessing =====
  bool loadModel(const std::string &path) {
    if (path.empty()) return false;

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(path, mesh) <= 0) {
      RCLCPP_ERROR(get_logger(), "Could not load STL file: %s", path.c_str());
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromPCLPointCloud2(mesh.cloud, tmp);
    model_raw_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(tmp);

    model_ds_ = voxelDown(model_raw_, std::max(0.01f, static_cast<float>(voxel_leaf_)));
    computeNormalsAndFPFH(model_ds_, model_normals_, model_fpfh_,
                          std::max<float>(2.f * voxel_leaf_, normal_radius_),
                          std::max<float>(4.f * voxel_leaf_, feature_radius_));
    return true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, float leaf) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
    return out;
  }

  void clampROI(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(static_cast<float>(roi_min_z_), static_cast<float>(roi_max_z_));
  pass.filter(*cloud);
  if (cloud->empty()) return;

  pass.setFilterFieldName("x");
  pass.setFilterLimits(static_cast<float>(-roi_half_xy_), static_cast<float>(roi_half_xy_));
  pass.filter(*cloud);
  if (cloud->empty()) return;

  pass.setFilterFieldName("y");
  pass.setFilterLimits(static_cast<float>(-roi_half_xy_), static_cast<float>(roi_half_xy_));
  pass.filter(*cloud);
  if (cloud->empty()) return;

  if (!remove_plane_ || cloud->size() < 2000) return;

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(200);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  seg.segment(*inliers, *coeff);

  // Only remove if the plane is not basically the whole cloud
  if (inliers && !inliers->indices.empty() && inliers->indices.size() < cloud->size()*0.8) {
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(inliers);
    ex.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    ex.filter(*filtered);
    cloud.swap(filtered);
  }
}

  void computeNormalsAndFPFH(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr pts,
      pcl::PointCloud<pcl::Normal>::Ptr &normals,
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh,
      float normal_radius, float feature_radius)
  {
    normals.reset(new pcl::PointCloud<pcl::Normal>);
    fpfh.reset(new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(pts);
    auto tree1 = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ne.setSearchMethod(tree1);
    ne.setRadiusSearch(normal_radius);
    ne.compute(*normals);

    pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fe;
    fe.setInputCloud(pts);
    fe.setInputNormals(normals);
    auto tree2 = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    fe.setSearchMethod(tree2);
    fe.setRadiusSearch(feature_radius);
    fe.compute(*fpfh);
  }

  void tick() {
    if (!has_inflight_) {
      if (!client_->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                             "Service %s not available yet.", service_name_.c_str());
        return;
      }
      auto req = std::make_shared<GetPointCloud::Request>();
      inflight_ = client_->async_send_request(req);
      has_inflight_ = true;
      inflight_start_ = now();
      return;
    }

    if (inflight_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      if ((now() - inflight_start_).seconds() > 3.0) {
        RCLCPP_WARN(get_logger(), "Service %s response timeout; restarting request.",
                    service_name_.c_str());
        has_inflight_ = false;
      }
      return;
    }

    auto resp = inflight_.get();
    has_inflight_ = false;

    const auto & ros_cloud = resp->cloud;
    if (ros_cloud.width == 0 || ros_cloud.data.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty cloud.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(ros_cloud, *scene);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*scene, *scene, idx);

    clampROI(scene);

    float leaf = std::max<float>(0.01f, voxel_leaf_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ds;
    try {
      scene_ds = voxelDown(scene, leaf);
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Voxel overflow at leaf=%.4f; retrying with x2", leaf);
      leaf *= 2.0f;
      scene_ds = voxelDown(scene, leaf);
    }
    if (scene_ds->empty()) {
      RCLCPP_WARN(get_logger(), "Downsampled scene is empty; skipping.");
      return;
    }

    if (scene_ds->size() > 120000) {
      std::mt19937 rng{std::random_device{}()};
      std::shuffle(scene_ds->points.begin(), scene_ds->points.end(), rng);
      scene_ds->points.resize(120000);
      scene_ds->width = 120000; scene_ds->height = 1;
    }

    float normal_r   = std::max<float>(2.f * leaf, normal_radius_);
    float feature_r  = std::max<float>(4.f * leaf, feature_radius_);
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_fpfh;
    computeNormalsAndFPFH(scene_ds, scene_normals, scene_fpfh, normal_r, feature_r);

    RCLCPP_INFO(get_logger(),
      "Registration input: model_ds=%zu, scene_ds=%zu, leaf=%.3f, nr=%.3f, fr=%.3f",
      model_ds_->size(), scene_ds->size(), leaf, normal_r, feature_r);

    pcl::SampleConsensusPrerejective<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> align;
    align.setInputSource(model_ds_);
    align.setSourceFeatures(model_fpfh_);
    align.setInputTarget(scene_ds);
    align.setTargetFeatures(scene_fpfh);
    align.setMaximumIterations(2000);
    align.setNumberOfSamples(4);
    align.setCorrespondenceRandomness(12);
    align.setSimilarityThreshold(0.90f);
    align.setMaxCorrespondenceDistance(3.0f * leaf);
    align.setInlierFraction(0.20f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    auto t0 = std::chrono::steady_clock::now();
    align.align(*aligned);
    auto t1 = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    if (!align.hasConverged() || ms > 3000) {
      RCLCPP_WARN(get_logger(), "Prerejective %s in %ld ms.",
                  align.hasConverged() ? "timed out" : "failed", ms);
      return;
    }

    Eigen::Matrix4f T = align.getFinalTransformation();

    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setInputSource(aligned);
    icp.setInputTarget(scene_ds);
    icp.setMaxCorrespondenceDistance(2.0f * leaf);
    icp.setMaximumIterations(40);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    pcl::PointCloud<pcl::PointXYZ> icp_out;
    icp.align(icp_out);
    if (icp.hasConverged()) {
      T = icp.getFinalTransformation() * T;
      RCLCPP_INFO(get_logger(), "ICP refined inlier RMSE ~ %.6f", icp.getFitnessScore());
    } else {
      RCLCPP_WARN(get_logger(), "ICP did not converge, using prerejective pose.");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = ros_cloud.header.frame_id;

    Eigen::Matrix3f R = T.block<3,3>(0,0);
    Eigen::Vector3f t = T.block<3,1>(0,3);
    Eigen::Quaternionf q(R);

    pose.pose.position.x = t.x();
    pose.pose.position.y = t.y();
    pose.pose.position.z = t.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    pose_pub_->publish(pose);
    RCLCPP_INFO(get_logger(), "Published pose_estimate.");
  }

private:
  std::string service_name_;
  std::string mesh_path_;
  double voxel_leaf_, normal_radius_, feature_radius_;
  double roi_min_z_, roi_max_z_, roi_half_xy_;
  bool remove_plane_;
  int min_scene_pts_, min_model_pts_, max_model_pts_;
  double roi_min_z_, roi_max_z_, roi_half_xy_;
  bool remove_plane_;

  rclcpp::Client<GetPointCloud>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_future<GetPointCloud::Response::SharedPtr> inflight_;
  bool has_inflight_ = false;
  rclcpp::Time inflight_start_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_raw_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_ds_;
  pcl::PointCloud<pcl::Normal>::Ptr   model_normals_;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PoseEstimatorNode>());
  } catch (const std::exception &e) {
  }
  rclcpp::shutdown();
  return 0;
}
