#ifndef GROUND_FILTER_MODELS_LINE_FIT_BIN_HPP_
#define GROUND_FILTER_MODELS_LINE_FIT_BIN_HPP_

// C/C++
#include <atomic>
#include <limits>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ground_filter
{

class Bin {
public:
  struct MinZPoint {
    MinZPoint() : z(0), d(0) {}
    MinZPoint(const double& d, const double& z) : z(z), d(d) {}
    bool operator==(const MinZPoint& comp) {return z == comp.z && d == comp.d;}

    double z;
    double d;
  };

private:

  std::atomic<bool> has_point_;
  std::atomic<double> min_z;
  std::atomic<double> min_z_range;

public:

  Bin();

  /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin& segment);

  void addPoint(const pcl::PointXYZ& point);

  void addPoint(const double& d, const double& z);

  MinZPoint getMinZPoint();

  inline bool hasPoint() {return has_point_;}

}; // end class Bin

}  // end namespace ground_filter

#endif
