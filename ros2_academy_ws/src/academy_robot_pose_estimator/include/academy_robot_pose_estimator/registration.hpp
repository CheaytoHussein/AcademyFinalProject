#pragma once
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct RegistrationParams {
  float voxel_leaf = 0.005f;          // 5 mm
  float normal_radius = 0.01f;        // 1 cm
  float feature_radius = 0.025f;      // 2.5 cm
  float sac_max_corr = 0.02f;         // SAC max correspondence distance
  int   sac_max_iter = 50000;         // RANSAC iterations
  int   sac_num_samples = 3;          // points to sample
  int   sac_corr_randomness = 5;      // k nearest features
  float sac_similarity = 0.9f;        // edge length similarity
  float sac_inlier_fraction = 0.25f;  // inlier fraction
  float icp_max_corr = 0.01f;         // ICP max correspondence distance
  int   icp_max_iter = 50;
  float icp_trans_eps = 1e-6f;
  float icp_fitness_eps = 1e-6f;
};

bool registerModelToScene(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& model,
                          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& scene,
                          Eigen::Matrix4f& result,
                          const RegistrationParams& params,
                          std::string* dbg = nullptr);
