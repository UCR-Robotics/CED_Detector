#ifndef MY_PERCEPTION_H_
#define MY_PERCEPTION_H_

#include <iostream>
#include <fstream>
#include <chrono>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>  // needed for custom L1/L2 distance; skip PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>  // needed for custom L1/L2 distance
#include <teaser/registration.h>
#include <yaml-cpp/yaml.h>

#include "sift_xyzrgbn.h"
#include "remove_nan.h"
#include "random_keypoint.h"
#include "ced_3d.h"
#include "ced_6d.h"


namespace my {

template <typename FeatureT>
class Perception
{
  public:
    using PointT = pcl::PointXYZRGBNormal;
    using NormalT = pcl::PointXYZRGBNormal;

    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
    using KeypointPtr = pcl::PointCloud<PointT>::Ptr;
    using FeaturePtr = typename pcl::PointCloud<FeatureT>::Ptr;
    using Visualizer = pcl::visualization::PCLVisualizer;

    Perception (const YAML::Node& node, std::string log_path = "/dev/null")
        : kdtree_ (new pcl::search::KdTree<PointT, pcl::KdTreeFLANN<PointT, flann::L2_Simple<float>>>),
          log_ (log_path, std::ofstream::app),
          node_ (node),
          voxel_resolution_ (node_["voxel_resolution"].as<float>()),
          verbose_ (node_["verbose"].as<bool>()),
          logging_ (node_["logging"].as<bool>()) {
      if (!log_.is_open()) std::cerr << "[ERROR] Cannot open file " << log_path << "\n";
    }

    ~Perception () {
      if (logging_) log_ << "\n";
    }

  public:
    void removeNaNPoints (const PointCloudPtr& cloud);
    void downSampleVoxelGrids (const PointCloudPtr& cloud);
    void estimateNormals (const PointCloudPtr& cloud);

    void detectKeypoints (const std::string& method, const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void extractFeatures_PFHRGB (const PointCloudPtr& cloud, const KeypointPtr& keypoint, const FeaturePtr& feature);
    void featureMatching (const KeypointPtr& src_keypoint, const KeypointPtr& tgt_keypoint,
                          const FeaturePtr& src_feature, const FeaturePtr& tgt_feature,
                          Eigen::Matrix3Xd& src_cloud, Eigen::Matrix3Xd& tgt_cloud);
    void teaserRegistration (const Eigen::Matrix3Xd& src_cloud, const Eigen::Matrix3Xd& tgt_cloud,
                             Eigen::Matrix4f& transformation);

    void visualizeKeypoints (const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void visualizeRegistration (const PointCloudPtr& source, const PointCloudPtr& source_transformed,
                                const PointCloudPtr& target);

  protected:
    void detectKeypoints_Random (const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void detectKeypoints_ISS (const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void detectKeypoints_SIFT (const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void detectKeypoints_Harris3D (const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void detectKeypoints_Harris6D (const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void detectKeypoints_CED3D (const PointCloudPtr& cloud, const KeypointPtr& keypoint);
    void detectKeypoints_CED6D (const PointCloudPtr& cloud, const KeypointPtr& keypoint);

    void removeRadiusOutlier (const PointCloudPtr& cloud);
    void timingStart ();
    float timingEnd ();

  private:
    pcl::search::KdTree<PointT, pcl::KdTreeFLANN<PointT, flann::L2_Simple<float>>>::Ptr kdtree_;
    std::chrono::steady_clock::time_point tic_;
    std::ofstream log_;
    YAML::Node node_;
    const float voxel_resolution_;
    const bool verbose_;
    bool logging_;
};

}  // namespace my

#include "impl/perception.hpp"

#endif  // MY_PERCEPTION_H_
