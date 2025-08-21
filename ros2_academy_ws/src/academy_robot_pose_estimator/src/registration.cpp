#include "academy_robot_pose_estimator/registration.hpp"
#include <sstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>

namespace {
template<typename CloudT>
void removeNaNs(const typename CloudT::ConstPtr& in, typename CloudT::Ptr& out) {
  std::vector<int> idx;
  out.reset(new CloudT);
  pcl::removeNaNFromPointCloud(*in, *out, idx);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float leaf) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(in);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
  return out;
}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in, float radius) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(in);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius);
  ne.compute(*normals);
  return normals;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& in,
    const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
    float radius) {
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr feats(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est;
  est.setInputCloud(in);
  est.setInputNormals(normals);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  est.setSearchMethod(tree);
  est.setRadiusSearch(radius);
  est.compute(*feats);
  return feats;
}
} // namespace

bool registerModelToScene(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& model,
                          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& scene,
                          Eigen::Matrix4f& result,
                          const RegistrationParams& p,
                          std::string* dbg) {
  std::ostringstream oss;

  // Clean
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_clean, scene_clean;
  removeNaNs<pcl::PointCloud<pcl::PointXYZ>>(model, model_clean);
  removeNaNs<pcl::PointCloud<pcl::PointXYZ>>(scene, scene_clean);

  // Downsample
  auto model_ds = voxelDown(model_clean, p.voxel_leaf);
  auto scene_ds = voxelDown(scene_clean, p.voxel_leaf);
  oss << "Model points (ds): " << model_ds->size() << ", Scene points (ds): " << scene_ds->size() << "\n";

  if (model_ds->empty() || scene_ds->empty()) {
    if (dbg) *dbg = oss.str() + "Empty downsampled clouds.";
    return false;
  }

  // Normals
  auto model_n = estimateNormals(model_ds, p.normal_radius);
  auto scene_n = estimateNormals(scene_ds, p.normal_radius);

  // Features
  auto model_f = computeFPFH(model_ds, model_n, p.feature_radius);
  auto scene_f = computeFPFH(scene_ds, scene_n, p.feature_radius);

  // RANSAC-based prerejective alignment
  pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
  align.setInputSource(model_ds);
  align.setSourceFeatures(model_f);
  align.setInputTarget(scene_ds);
  align.setTargetFeatures(scene_f);
  align.setMaximumIterations(p.sac_max_iter);
  align.setNumberOfSamples(p.sac_num_samples);
  align.setCorrespondenceRandomness(p.sac_corr_randomness);
  align.setSimilarityThreshold(p.sac_similarity);
  align.setInlierFraction(p.sac_inlier_fraction);
  align.setMaxCorrespondenceDistance(p.sac_max_corr);

  pcl::PointCloud<pcl::PointXYZ> model_aligned;
  align.align(model_aligned);

  if (!align.hasConverged()) {
    if (dbg) *dbg = oss.str() + "Prerejective alignment failed.";
    return false;
  }
  Eigen::Matrix4f sac_tf = align.getFinalTransformation();
  oss << "SAC fitness (inliers): " << align.getInliers().size() << "\n";

  // ICP refine
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(p.icp_max_iter);
  icp.setMaxCorrespondenceDistance(p.icp_max_corr);
  icp.setTransformationEpsilon(p.icp_trans_eps);
  icp.setEuclideanFitnessEpsilon(p.icp_fitness_eps);
  icp.setInputSource(model_ds);
  icp.setInputTarget(scene_ds);

  pcl::PointCloud<pcl::PointXYZ> icp_out;
  icp.align(icp_out, sac_tf);
  if (!icp.hasConverged()) {
    if (dbg) *dbg = oss.str() + "ICP failed.";
    return false;
  }
  result = icp.getFinalTransformation();
  oss << "ICP fitness: " << icp.getFitnessScore() << "\n";

  if (dbg) *dbg = oss.str();
  return true;
}
