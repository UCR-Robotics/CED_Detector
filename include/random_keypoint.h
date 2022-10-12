#ifndef MY_RANDOM_KEYPOINT_H_
#define MY_RANDOM_KEYPOINT_H_

#include <random>

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  template <typename PointInT, typename PointOutT>
  class RandomKeypoint : public Keypoint<PointInT, PointOutT>
  {
    public:
      using Ptr = boost::shared_ptr<RandomKeypoint<PointInT, PointOutT> >;
      using ConstPtr = boost::shared_ptr<const RandomKeypoint<PointInT, PointOutT> >;

      using PointCloudIn = typename Keypoint<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Keypoint<PointInT, PointOutT>::PointCloudOut;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      /** @brief Constructor. */
      RandomKeypoint ()
      : num_keypoints_ (300)
      {
        name_ = "RandomKeypoint";
      }

      /** @brief Destructor. */
      ~RandomKeypoint () {}

      /**
       * @brief Set the number of keypoints to be generated at random.
       * @param num_keypoints the number of keypoints to be generated
       */
      inline void
      setNumKeypoints (int num_keypoints) { num_keypoints_ = num_keypoints; }

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

      /** @brief The number of keypoints to be generated at random. */
      int num_keypoints_;
  };

}  // namespace pcl

#include "impl/random_keypoint.hpp"

#endif   // MY_RANDOM_KEYPOINT_H_
