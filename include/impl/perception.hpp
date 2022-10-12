#ifndef MY_PERCEPTION_HPP_
#define MY_PERCEPTION_HPP_

template <typename FeatureT>
void my::Perception<FeatureT>::timingStart () {
  tic_ = std::chrono::steady_clock::now();
}


template <typename FeatureT>
float my::Perception<FeatureT>::timingEnd () {
  auto toc = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>(toc - tic_).count() / 1000000.0;
}


template <typename FeatureT>
void my::Perception<FeatureT>::removeNaNPoints (const PointCloudPtr& cloud) {
  timingStart();
  std::vector<int> nan_idx;
  pcl::removeNaNFromPointCloudBruteForce (*cloud, *cloud, nan_idx);
  if (verbose_) std::cout << "Contained " << cloud->size () << " points after removing NaN points\n";
  pcl::removeNaNRGBFromPointCloud (*cloud, *cloud, nan_idx);
  if (verbose_) std::cout << "Contained " << cloud->size () << " points after removing NaN RGB points\n";
  if (logging_) log_ << "removeNaN, " << cloud->size() << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::removeRadiusOutlier (const PointCloudPtr& cloud) {
  // timingStart();
  pcl::RadiusOutlierRemoval<PointT> filter (false);
  filter.setInputCloud (cloud);
  filter.setRadiusSearch (0.5);
  filter.setMinNeighborsInRadius (2);
  filter.filter (*cloud);
  // if (verbose_) std::cout << "Contained " << cloud->size() << " points after removing outliers by radius\n";
  // if (logging_) log_ << "removeRadiusOutlier, " << cloud->size() << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::downSampleVoxelGrids (const PointCloudPtr& cloud) {
  timingStart();
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize (voxel_resolution_, voxel_resolution_, voxel_resolution_);
  filter.setInputCloud (cloud);
  filter.filter (*cloud);
  if (verbose_) std::cout << "Downsampled to " << cloud->size () << " points\n";
  if (logging_) log_ << "downsample, " << cloud->size() << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::estimateNormals (const PointCloudPtr& cloud) {
  timingStart();
  // Compute surface normals
  pcl::NormalEstimation<PointT, NormalT> ne;
  ne.setSearchMethod (kdtree_);
  ne.setRadiusSearch (node_["normal_est_radius"].as<float> ());
  ne.setInputCloud (cloud);
  ne.compute (*cloud);
  if (verbose_) std::cout << "Computed " << cloud->size () << " Normals\n";

  // Remove NaN normals from point clouds
  std::vector<int> nan_idx;
  pcl::removeNaNNormalsFromPointCloud (*cloud, *cloud, nan_idx);
  if (verbose_) std::cout << "Contained " << cloud->size () << " points after removing NaN normals\n";
  if (logging_) log_ << "estimateNormals, " << cloud->size() << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints (const std::string& keypoint_method, 
                                                const PointCloudPtr& cloud,
                                                const KeypointPtr& keypoint) {
  if (keypoint_method == "random")
    detectKeypoints_Random (cloud, keypoint);
  else if (keypoint_method == "iss")
    detectKeypoints_ISS (cloud, keypoint);
  else if (keypoint_method == "sift")
    detectKeypoints_SIFT (cloud, keypoint);
  else if (keypoint_method == "harris3d")
    detectKeypoints_Harris3D (cloud, keypoint);
  else if (keypoint_method == "harris6d")
    detectKeypoints_Harris6D (cloud, keypoint);
  else if (keypoint_method == "ced3d")
    detectKeypoints_CED3D (cloud, keypoint);
  else if (keypoint_method == "ced6d")
    detectKeypoints_CED6D (cloud, keypoint);
  else
    PCL_ERROR("no keypoint method is selected");
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints_Random (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  timingStart();
  // Estimate Random keypoints
  pcl::RandomKeypoint<PointT, PointT> random;
  random.setSearchMethod (kdtree_);
  random.setNumKeypoints (node_["num_random_keypoints"].as<int> ());
  random.setInputCloud (cloud);
  random.compute (*keypoint);
  if (verbose_) std::cout << "Computed " << keypoint->points.size () << " Random keypoints\n";
  if (logging_) log_ << "random, " << keypoint->points.size () << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints_ISS (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  timingStart();
  // Estimate ISS keypoints
  pcl::ISSKeypoint3D<PointT, PointT, NormalT> iss;
  iss.setSearchMethod (kdtree_);
  iss.setSalientRadius (node_["keypoint_search_radius"].as<float> ());
  iss.setNonMaxRadius (node_["non_max_radius"].as<float> ());
  iss.setThreshold21 (node_["iss_gamma21"].as<float> ());
  iss.setThreshold32 (node_["iss_gamma32"].as<float> ());
  iss.setMinNeighbors (node_["min_neighbors"].as<int> ());
  iss.setNumberOfThreads (node_["num_thread"].as<int> ());
  iss.setInputCloud (cloud);
  // iss.setNormals (cloud);
  iss.compute (*keypoint);
  if (verbose_) std::cout << "Computed " << keypoint->points.size () << " ISS keypoints\n";
  if (logging_) log_ << "iss, " << keypoint->points.size () << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints_SIFT (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  timingStart();
  // SIFT Keypoint parameters
  float min_scale = node_["sift_min_scale"].as<float> ();
  int n_octaves = node_["sift_n_octaves"].as<int> (); 
  int n_scales_per_octave = node_["sift_n_scales_per_octave"].as<int> ();
  float min_contrast = node_["sift_min_contrast"].as<float> ();

  // Make sure normals are ready (required by SIFT)
  logging_ = false;
  estimateNormals (cloud);
  logging_ = true;
  // Estimate SIFT keypoints
  pcl::SIFTKeypoint<NormalT, PointT> sift;
  sift.setSearchMethod (kdtree_);
  sift.setScales (min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast (min_contrast);
  sift.setInputCloud (cloud);
  sift.compute (*keypoint);
  if (verbose_) std::cout << "Computed " << keypoint->points.size () << " SIFT keypoints\n";
  if (logging_) log_ << "sift, " << keypoint->points.size () << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints_Harris3D (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  timingStart();
  // Estimate Harris3D keypoints
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_keypoint (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI, NormalT> harris3d;
  harris3d.setNonMaxSupression(true);
  harris3d.setThreshold(node_["harris_threshold"].as<float> ());
  harris3d.setSearchMethod (kdtree_);
  harris3d.setRadiusSearch (node_["keypoint_search_radius"].as<float> ());
  harris3d.setNumberOfThreads (node_["num_thread"].as<int> ());
  harris3d.setInputCloud (cloud);
  // harris3d.setNormals (cloud);
  harris3d.compute (*temp_keypoint);
  pcl::copyPointCloud(*temp_keypoint, *keypoint);
  if (verbose_) std::cout << "Computed " << keypoint->points.size () << " Harris3D keypoints\n";
  if (logging_) log_ << "harris3d, " << keypoint->points.size () << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints_Harris6D (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  timingStart();
  // Estimate Harris6D keypoints
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_keypoint (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::HarrisKeypoint6D<PointT, pcl::PointXYZI> harris6d;
  harris6d.setNonMaxSupression(true);
  harris6d.setThreshold(node_["harris_threshold"].as<float> ());
  harris6d.setSearchMethod (kdtree_);
  harris6d.setRadiusSearch (node_["keypoint_search_radius"].as<float> ());
  harris6d.setNumberOfThreads (node_["num_thread"].as<int> ());
  harris6d.setInputCloud (cloud);
  harris6d.compute (*temp_keypoint);

  // There is a bug in Harris6D algorithm, which produces NaN points but still sets is_dense=true)
  std::vector<int> nan_idx;
  pcl::removeNaNFromPointCloudBruteForce (*temp_keypoint,*temp_keypoint, nan_idx);
  pcl::copyPointCloud(*temp_keypoint, *keypoint);
  removeRadiusOutlier(keypoint);  // remove outliers located in the middle of nowhere
  if (verbose_) std::cout << "Computed " << keypoint->points.size () << " Harris6D keypoints\n";
  if (logging_) log_ << "harris6d, " << keypoint->points.size () << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints_CED3D (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  timingStart();
  // Estimate CED-3D keypoints
  pcl::CEDKeypoint3D<PointT, PointT> ced3d;
  ced3d.setSearchMethod (kdtree_);
  ced3d.setRadiusSearch (node_["keypoint_search_radius"].as<float> ());
  ced3d.setNonMaxRadius (node_["non_max_radius"].as<float> ());
  ced3d.setCentroidThreshold (node_["ced3d_centroid"].as<float> ());
  ced3d.setMinNeighbors (node_["min_neighbors"].as<int> ());
  ced3d.setInputCloud (cloud);
  ced3d.compute (*keypoint);
  if (verbose_) std::cout << "Computed " << keypoint->points.size () << " CED-3D keypoints\n";
  if (logging_) log_ << "ced3d, " << keypoint->points.size () << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::detectKeypoints_CED6D (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  timingStart();
  // Estimate CED-6D keypoints
  pcl::CEDKeypoint6D<PointT, PointT> ced6d;
  ced6d.setSearchMethod (kdtree_);
  ced6d.setRadiusSearch (node_["keypoint_search_radius"].as<float> ());
  ced6d.setNonMaxRadius (node_["non_max_radius"].as<float> ());
  ced6d.setCentroidThreshold (node_["ced6d_centroid"].as<float> ());
  ced6d.setCentroidThresholdRGB (node_["ced6d_centroid_rgb"].as<float> ());
  ced6d.setMinNeighbors (node_["min_neighbors"].as<int> ());
  ced6d.setInputCloud (cloud);
  ced6d.compute (*keypoint);
  if (verbose_) std::cout << "Computed " << keypoint->points.size () << " CED-6D keypoints\n";
  if (logging_) log_ << "ced6d, " << keypoint->points.size () << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::extractFeatures_PFHRGB (const PointCloudPtr& cloud, const KeypointPtr& keypoint,
                                                       const FeaturePtr& feature) {
  timingStart();
  // Extract PFHRGB features
  pcl::PFHRGBEstimation<PointT, NormalT> pfhrgb;
  pfhrgb.setSearchMethod (kdtree_);
  pfhrgb.setRadiusSearch (node_["feature_search_radius"].as<float> ());
  pfhrgb.setInputCloud (keypoint);
  pfhrgb.setSearchSurface (cloud);
  pfhrgb.setInputNormals (cloud);
  pfhrgb.compute (*feature);
  if (verbose_) std::cout << "Computed " << feature->size () << " PFHRGB features\n";
  if (logging_) log_ << "pfhrgb, " << feature->size() << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::featureMatching (const KeypointPtr& src_keypoint, const KeypointPtr& tgt_keypoint,
                                                const FeaturePtr& src_feature, const FeaturePtr& tgt_feature,
                                                Eigen::Matrix3Xd& src_cloud, Eigen::Matrix3Xd& tgt_cloud) {
  timingStart();
  // Initialization
  boost::shared_ptr<pcl::KdTreeFLANN<FeatureT, flann::L1<float> > > feature_tree
      (new pcl::KdTreeFLANN<FeatureT, flann::L1<float> >);
  boost::shared_ptr<pcl::KdTreeFLANN<FeatureT, flann::L1<float> > > feature_tree_reciprocal
      (new pcl::KdTreeFLANN<FeatureT, flann::L1<float> >);
  feature_tree->setInputCloud (tgt_feature);
  feature_tree_reciprocal->setInputCloud (src_feature);

  // Find correspondences
  std::vector<int> src_indices;
  std::vector<int> tgt_indices;
  std::vector<int> index (1);
  std::vector<float> sqr_distance (1);
  std::vector<int> index_reciprocal (1);
  std::vector<float> sqr_dist_reciprocal (1);
  int target_idx = 0;
  // Iterate over the input set of source indices
  for (int idx = 0; idx != src_feature->points.size (); ++idx)
  {
    feature_tree->nearestKSearch (src_feature->points[idx], 1, index, sqr_distance);
    target_idx = index[0];

    feature_tree_reciprocal->nearestKSearch (tgt_feature->points[target_idx], 1, index_reciprocal, sqr_dist_reciprocal);
    if (idx != index_reciprocal[0])
      continue;
    src_indices.push_back (idx);
    tgt_indices.push_back (index[0]);
  }
  int num_correspondence = src_indices.size ();
  if (verbose_) std::cout << "Number of correspondences found: " << num_correspondence << std::endl;

  // Prepare data in Eigen
  src_cloud.resize (3, num_correspondence);
  tgt_cloud.resize (3, num_correspondence);
  for (int i = 0; i < num_correspondence; ++i) {
    int src_i = src_indices[i];
    int tgt_i = tgt_indices[i];
    src_cloud.col(i) << src_keypoint->points[src_i].x, src_keypoint->points[src_i].y, src_keypoint->points[src_i].z;
    tgt_cloud.col(i) << tgt_keypoint->points[tgt_i].x, tgt_keypoint->points[tgt_i].y, tgt_keypoint->points[tgt_i].z;
  }
  if (logging_) log_ << "featureMatching, " << num_correspondence << ", " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::teaserRegistration (const Eigen::Matrix3Xd& src_cloud,
                                                   const Eigen::Matrix3Xd& tgt_cloud,
                                                   Eigen::Matrix4f& transformation) {
  timingStart();
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  solver.solve(src_cloud, tgt_cloud);
  auto solution = solver.getSolution();
  transformation.topLeftCorner(3, 3) = solution.rotation.cast<float>();
  transformation.topRightCorner(3, 1) = solution.translation.cast<float>();
  if (verbose_) std::cout << "Estimated transformation " << std::endl << transformation << std::endl;
  if (logging_) log_ << "teaserRegistration, " << timingEnd() << ", ";
}


template <typename FeatureT>
void my::Perception<FeatureT>::visualizeKeypoints (const PointCloudPtr& cloud, const KeypointPtr& keypoint) {
  // Add point clouds to the viewer
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.addPointCloud<PointT> (cloud, "cloud");

  // As a workaround, we use normals to represent keypoints
  pcl::NormalEstimation<PointT, NormalT> ne;
  ne.setSearchMethod (kdtree_);
  ne.setKSearch (20); // estimate normal using the closest 20 neighbor points
  ne.setInputCloud (keypoint);
  ne.compute (*keypoint);
  std::vector<int> nan_idx;
  pcl::removeNaNNormalsFromPointCloud (*keypoint, *keypoint, nan_idx);
  visualizer.addPointCloudNormals<NormalT> (keypoint, 1, 0.03f, "keypoint");

  while (!visualizer.wasStopped ()) {
    visualizer.spinOnce ();
    pcl_sleep(0.01);
  }
}


template <typename FeatureT>
void my::Perception<FeatureT>::visualizeRegistration (const PointCloudPtr& source,
                                                      const PointCloudPtr& source_transformed, 
                                                      const PointCloudPtr& target) {
  // Add point clouds to the viewer
  pcl::visualization::PCLVisualizer visualizer;
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_color_handler (source, 255, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> src_trans_color_handler (source_transformed, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_color_handler (target, 0, 255, 255);
  visualizer.addPointCloud (source, src_color_handler, "source cloud");
  visualizer.addPointCloud (source_transformed, src_trans_color_handler, "source cloud transformed");
  visualizer.addPointCloud (target, tgt_color_handler, "target cloud");
  
  while (!visualizer.wasStopped ()) {
    visualizer.spinOnce ();
    pcl_sleep(0.01);
  }
}


#endif  // MY_PERCEPTION_HPP_
