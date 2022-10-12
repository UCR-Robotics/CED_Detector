#ifndef MY_SIFT_XYZRGBN_H_
#define MY_SIFT_XYZRGBN_H_

namespace pcl
{
  template<>
  struct SIFTKeypointFieldSelector<PointXYZRGBNormal>
  {
    inline float
    operator () (const PointXYZRGBNormal & p) const
    {
      return p.curvature;
      // return (static_cast<float> (299*p.r + 587*p.g + 114*p.b) / 1000.0f);
    }
  };
}


#endif  // MY_SIFT_XYZRGBN_H_
