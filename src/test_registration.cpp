#include "perception.h"

using PointT = pcl::PointXYZRGBNormal;
using FeatureT = pcl::PFHRGBSignature250;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: ./test_registration <config.yaml> " << std::endl;
    return 0;
  }

  // parse configuration
  YAML::Node root_node = YAML::LoadFile(argv[1]);
  YAML::Node test_reg_node = root_node["test_reg_config"];
  YAML::Node perception_node = root_node["perception_config"];

  const std::string keypoint_method = test_reg_node["keypoint_method"].as<std::string> ();
  const std::string src_filename = test_reg_node["source_cloud"].as<std::string> ();
  const std::string tar_filename = test_reg_node["target_cloud"].as<std::string> ();
  const std::string log_filename = test_reg_node["log_filename"].as<std::string> ();

  // prepare point clouds
  pcl::PointCloud<PointT>::Ptr source_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr target_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr source_keypoints (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr target_keypoints (new pcl::PointCloud<PointT>);
  pcl::PointCloud<FeatureT>::Ptr source_features (new pcl::PointCloud<FeatureT>);
  pcl::PointCloud<FeatureT>::Ptr target_features (new pcl::PointCloud<FeatureT>);
  pcl::PointCloud<PointT>::Ptr source_cloud_transformed (new pcl::PointCloud<PointT>);
  Eigen::Matrix3Xd src_cloud;
  Eigen::Matrix3Xd tgt_cloud;
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

  if (pcl::io::loadPCDFile (src_filename, *source_cloud) == -1)
    return 0;
  else if (perception_node["verbose"].as<bool> ())
    std::cout << "Loaded " << source_cloud->size() << " points for source cloud" << std::endl;
  if (pcl::io::loadPCDFile (tar_filename, *target_cloud) == -1)
    return 0;
  else if (perception_node["verbose"].as<bool> ())
    std::cout << "Loaded " << target_cloud->size() << " points for target cloud" << std::endl;

  // run algorithms
  my::Perception<FeatureT> perception(perception_node, log_filename);

  perception.removeNaNPoints (source_cloud);
  perception.removeNaNPoints (target_cloud);

  if (perception_node["downsample"].as<bool> ()) {
    perception.downSampleVoxelGrids (source_cloud);
    perception.downSampleVoxelGrids (target_cloud);
  }

  if (perception_node["estimate_normal"].as<bool> ()) {
    perception.estimateNormals (source_cloud);
    perception.estimateNormals (target_cloud);
  }

  perception.detectKeypoints(keypoint_method, source_cloud, source_keypoints);
  perception.detectKeypoints(keypoint_method, target_cloud, target_keypoints);

  perception.extractFeatures_PFHRGB (source_cloud, source_keypoints, source_features);
  perception.extractFeatures_PFHRGB (target_cloud, target_keypoints, target_features);

  perception.featureMatching(source_keypoints, target_keypoints, source_features, target_features, src_cloud, tgt_cloud);
  perception.teaserRegistration (src_cloud, tgt_cloud, transformation);

  // visualization
  pcl::transformPointCloudWithNormals (*source_cloud, *source_cloud_transformed, transformation);
  perception.visualizeRegistration(source_cloud, source_cloud_transformed, target_cloud);
}
