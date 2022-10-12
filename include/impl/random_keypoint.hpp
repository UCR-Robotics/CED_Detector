#ifndef MY_RANDOM_KEYPOINT_HPP_
#define MY_RANDOM_KEYPOINT_HPP_


template<typename PointInT, typename PointOutT> bool
pcl::RandomKeypoint<PointInT, PointOutT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }
  return (true);
}


template<typename PointInT, typename PointOutT> void
pcl::RandomKeypoint<PointInT, PointOutT>::detectKeypoints (PointCloudOut &output)
{
  // initialization
  output.points.clear ();
  const int input_size = static_cast<int> (input_->points.size ());

  // random seed
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1, input_size);

  // pick keypoints at random
  for (int i = 1; i <= num_keypoints_; ++i)
  {
    int index = dis(gen);
    PointOutT p;
    p.getVector3fMap () = input_->points[index].getVector3fMap ();
    output.points.push_back (p);
    keypoints_indices_->indices.push_back (index);
  }

  // output
  output.height = 1;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.is_dense = input_->is_dense;
}


#endif   // MY_RANDOM_KEYPOINT_HPP_
