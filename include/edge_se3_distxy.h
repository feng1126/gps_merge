#ifndef EDGE_SE3_EXPMAP_DISTXY_H
#define EDGE_SE3_EXPMAP_DISTXY_H

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o
{

class EdgePointDist : public BaseBinaryEdge<1, double,  g2o::VertexSE3, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgePointDist()
  {
  }

  void computeError() override
  {
  
   const g2o::VertexSE3 *v1 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
   const g2o::VertexSE3 *v2 = static_cast<const g2o::VertexSE3 *>(_vertices[1]);
    Eigen::Vector3d p1 = v1->estimate().translation();
    Eigen::Vector3d p2 = v2->estimate().translation();
    Eigen::Vector3d dist_ = p2 - p1;

    double estim_dist = sqrt(pow(dist_[0], 2)  + pow(dist_[2], 2));

    _error[0] = estim_dist - _measurement;
  }

  void setMeasurement(const double &m) override
  {
    _measurement = m;
  }
  virtual bool read(std::istream & /*is*/)
  {
    return false;
  }

  virtual bool write(std::ostream & /*os*/) const
  {
    return false;
  }
};


} // namespace g2o
#endif
