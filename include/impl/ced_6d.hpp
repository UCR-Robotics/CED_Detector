#ifndef MY_CED_6D_HPP_
#define MY_CED_6D_HPP_


template<typename PointInT, typename PointOutT> bool
pcl::CEDKeypoint6D<PointInT, PointOutT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }
  if (non_max_radius_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the non maxima radius (%f) must be strict positive!\n",
		name_.c_str (), non_max_radius_);
    return (false);
  }
  if (min_neighbors_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the minimum number of neighbors (%f) must be strict positive!\n",
		name_.c_str (), min_neighbors_);
    return (false);
  }
  if (!pcl::traits::has_color<PointInT>::value)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : this point cloud has no color!\n",
    name_.c_str ());
    return (false);
  }
  return (true);
}


template<typename PointInT, typename PointOutT> void
pcl::CEDKeypoint6D<PointInT, PointOutT>::detectKeypoints (PointCloudOut &output)
{
  // initialization
  output.points.clear ();
  const int input_size = static_cast<int> (input_->points.size ());
  std::vector<float> geo_centroid_norm (input_size);
  std::vector<float> rgb_centroid_norm (input_size);

  // compute the norm of centroid vector
  for (int idx = 0; idx < input_size; ++idx)
  {
    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    tree_->radiusSearch (idx, search_radius_, nn_indices, nn_dists);

    float nn_size = nn_indices.size ();
    if (nn_size < min_neighbors_)
    {
      geo_centroid_norm[idx] = 0;
      rgb_centroid_norm[idx] = 0;
      continue;
    }

    Eigen::Vector3f geo_centroid = Eigen::Vector3f::Zero ();
    Eigen::Vector3i rgb_centroid = Eigen::Vector3i::Zero ();
    for (const auto& nn_idx : nn_indices)
    {
      geo_centroid += input_->points[nn_idx].getVector3fMap ();
      rgb_centroid += input_->points[nn_idx].getRGBVector3i ();
    }
    geo_centroid /= nn_size;
    geo_centroid_norm[idx] = (geo_centroid - input_->points[idx].getVector3fMap ()).norm ();
    Eigen::Vector3f rgb = rgb_centroid.cast<float> () / nn_size;
    Eigen::Vector3i rgb_idx = input_->points[idx].getRGBVector3i ();
    rgb_centroid_norm[idx] = (rgb - rgb_idx.cast<float> ()).lpNorm<1> () / 255;
  }

  // non-maximum suppression
  for (int idx = 0; idx < input_size; ++idx)
  {
    if (geo_centroid_norm[idx] < centroid_threshold_ * search_radius_
        && rgb_centroid_norm[idx] < centroid_threshold_rgb_)
      continue;

    std::vector<int> nn_indices;
    std::vector<float> nn_dists;
    tree_->radiusSearch (idx, non_max_radius_, nn_indices, nn_dists);

    bool maximum = true;
    for (const auto& nn_idx : nn_indices)
    {
      if (geo_centroid_norm[idx] * rgb_centroid_norm[idx] < geo_centroid_norm[nn_idx] * rgb_centroid_norm[nn_idx])
      {
        maximum = false;
        break;
      }
    }
    if (maximum)
    {
      PointOutT p;
      p.getVector3fMap () = input_->points[idx].getVector3fMap ();
      output.points.push_back (p);
      keypoints_indices_->indices.push_back (idx);
    }
  }

  // output
  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.is_dense = input_->is_dense;
}


#endif   // MY_CED_6D_HPP_
