#ifndef MY_REMOVE_NAN_H_
#define MY_REMOVE_NAN_H_

#include <pcl/pcl_macros.h>  // for pcl_isfinite -- PCL v1.8
//#include <pcl/common/point_tests.h> // for pcl::isFinite -- PCL v1.11
#include <pcl/filters/filter.h>


namespace pcl
{
  template <typename PointT>
  void removeNaNFromPointCloudBruteForce (const pcl::PointCloud<PointT> &cloud_in,
                                          pcl::PointCloud<PointT> &cloud_out,
                                          std::vector<int> &index);

  template <typename PointT>
  void removeNaNRGBFromPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                                   pcl::PointCloud<PointT> &cloud_out,
                                   std::vector<int> &index);
}


template <typename PointT>
void pcl::removeNaNFromPointCloudBruteForce (const pcl::PointCloud<PointT> &cloud_in,
                                             pcl::PointCloud<PointT> &cloud_out,
                                             std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // Check NaN anyway, regardless of the density of the data
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    if (!pcl_isfinite (cloud_in.points[i].x) ||
        !pcl_isfinite (cloud_in.points[i].y) ||
        !pcl_isfinite (cloud_in.points[i].z))
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    index[j] = static_cast<int>(i);
    j++;
  }
  if (j != cloud_in.points.size ())
  {
    // Resize to the correct size
    cloud_out.points.resize (j);
    index.resize (j);
  }

  cloud_out.height = 1;
  cloud_out.width  = static_cast<uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud_out.is_dense = true;
}


template <typename PointT>
void pcl::removeNaNRGBFromPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                                      pcl::PointCloud<PointT> &cloud_out,
                                      std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // Check NaN anyway, regardless of the density of the data
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    if (cloud_in.points[i].rgba == 0)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    index[j] = static_cast<int>(i);
    j++;
  }
  if (j != cloud_in.points.size ())
  {
    // Resize to the correct size
    cloud_out.points.resize (j);
    index.resize (j);
  }

  cloud_out.height = 1;
  cloud_out.width  = static_cast<uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud_out.is_dense = true;
}


#endif  // MY_REMOVE_NAN_H_
