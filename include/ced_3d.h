#ifndef MY_CED_3D_H_
#define MY_CED_3D_H_

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  /**
   * @brief CEDKeypoint3D is the geometry-only variant of the CEDKeypoint6D detector. This method does not require
   * point normal or eigenvalue decomposition.
   * 
   * For more information, please see:
   * 
   * @par
   * H. Teng, D. Chatziparaschis, X. Kan, A. K. Roy-Chowdhury and K. Karydis, "Centroid Distance Keypoint Detector for 
   * Colored Point Clouds", in IEEE/CVF Winter Conference on Applications of Computer Vision (WACV), 2023.
   * 
   * Example code:
   * 
   * @code
   * using PointT = pcl::PointXYZ
   * pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
   * pcl::PointCloud<PointT>::Ptr keypoints (new pcl::PointCloud<PointT>);
   * pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>);
   * 
   * // load point cloud data
   * // remove NaN points (optional, but recommended)
   * // downsample to a reasonable resolution (optional, but recommended)
   * 
   * pcl::CEDKeypoint3D<PointT, PointT> ced3d;
   * ced3d.setSearchMethod (kdtree);
   * ced3d.setRadiusSearch (0.05);      // meter, assuming cloud resolution is 0.01m
   * ced3d.setNonMaxRadius (0.05);      // meter, assuming cloud resolution is 0.01m
   * ced3d.setCentroidThreshold (0.2);  // unitless, the ratio of centroid distance to search radius in 3D space
   * ced3d.setMinNeighbors (5);         // number of neighbor points required for computing centroid
   * ced3d.setInputCloud (cloud);
   * ced3d.compute (*keypoints);
   * @endcode
   * 
   * @author Hanzhe Teng
   * @date Feb 10, 2021
   */
  template <typename PointInT, typename PointOutT>
  class CEDKeypoint3D : public Keypoint<PointInT, PointOutT>
  {
    public:
      using Ptr = boost::shared_ptr<CEDKeypoint3D<PointInT, PointOutT> >;
      using ConstPtr = boost::shared_ptr<const CEDKeypoint3D<PointInT, PointOutT> >;

      using PointCloudIn = typename Keypoint<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Keypoint<PointInT, PointOutT>::PointCloudOut;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      /** @brief Constructor. */
      CEDKeypoint3D ()
        : centroid_threshold_ (0.4)
        , non_max_radius_ (0.0)
        , min_neighbors_ (5)
      {
        name_ = "CEDKeypoint3D";
        non_max_radius_ = search_radius_;
      }

      /** @brief Destructor. */
      ~CEDKeypoint3D () {}

      /**
       * @brief Set the threshold for selecting geometry-salient keypoints in 3D space.
       * @param centroid_threshold threshold on the ratio of centroid distance to search radius in 3D space; unitless,
       * in the range of [0,1].
       */
      inline void
      setCentroidThreshold (double centroid_threshold) { centroid_threshold_ = centroid_threshold; }

      /**
       * @brief Set the radius for the application of non-maximum supression algorithm.
       * @param non_max_radius the non-maximum suppression radius
       */
      inline void
      setNonMaxRadius (double non_max_radius) { non_max_radius_ = non_max_radius; }

      /**
       * @brief Set the minimum number of neighbors required to be found for computing centroid.
       * @param min_neighbors the minimum number of neighbor points
       */
      inline void
      setMinNeighbors (int min_neighbors) { min_neighbors_ = min_neighbors; }

    protected:
      /**
       * @brief Perform initial checks before computing keypoints.
       * @return true if all the checks are passed, false otherwise
       */
      bool
      initCompute ();

      /**
       * @brief Detect keypoints.
       * @param output the cloud of detected keypoints
       */
      void
      detectKeypoints (PointCloudOut &output);

      /**
       * @brief Threshold on the ratio of centroid distance to search radius in 3D space, for selecting 
       * geometry-salient keypoints; unitless, in the range of [0,1].
       */
      double centroid_threshold_;

      /** @brief The non-maximum suppression radius. */
      double non_max_radius_;

      /** @brief The minimum number of neighbors required to be found for computing centroid. */
      int min_neighbors_;
  };

}  // namespace pcl

#include "impl/ced_3d.hpp"

#endif  // MY_CED_3D_H_
