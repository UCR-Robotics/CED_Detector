#include "perception.h"

using PointT = pcl::PointXYZRGBNormal;
using FeatureT = pcl::PFHRGBSignature250;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: ./test_keypoint <config.yaml>" << std::endl;
    return 0;
  }

  // parse configuration
  YAML::Node root_node = YAML::LoadFile(argv[1]);
  YAML::Node test_kp_node = root_node["test_kp_config"];
  YAML::Node perception_node = root_node["perception_config"];

  const std::string keypoint_method = test_kp_node["keypoint_method"].as<std::string> ();
  const std::string src_filename = test_kp_node["source_cloud"].as<std::string> ();
  const std::string log_filename = test_kp_node["log_filename"].as<std::string> ();
  const std::string save_cloud_to_file = test_kp_node["save_cloud_to_file"].as<std::string> ();
  const std::string save_keypoint_to_file = test_kp_node["save_keypoint_to_file"].as<std::string> ();
  const bool pcl_visualization = test_kp_node["pcl_visualization"].as<bool> ();

  // prepare point clouds
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr keypoint (new pcl::PointCloud<PointT>);
  if (pcl::io::loadPCDFile (src_filename, *cloud) == -1)
    return 0;
  else if (perception_node["verbose"].as<bool> ())
    std::cout << "Loaded " << cloud->size() << " points for source cloud" << std::endl;

  // run algorithms
  my::Perception<FeatureT> perception(perception_node, log_filename);
  perception.removeNaNPoints (cloud);
  if (perception_node["downsample"].as<bool> ())
    perception.downSampleVoxelGrids (cloud);
  if (perception_node["estimate_normal"].as<bool> ())
    perception.estimateNormals (cloud);
  perception.detectKeypoints(keypoint_method, cloud, keypoint);

  // visualization
  if (pcl_visualization)  // not recommended; better to use Open3D for keypoint visualization
    perception.visualizeKeypoints (cloud, keypoint);
  else {
    pcl::io::savePCDFile (save_cloud_to_file, *cloud);
    pcl::io::savePCDFile (save_keypoint_to_file, *keypoint);
  }
}
