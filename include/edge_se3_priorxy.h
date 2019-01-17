#ifndef EDGE_EXPMAP_PRIORXY_H
#define EDGE_EXPMAP_PRIORXY_H

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o
{


class EdgePointPriorXY : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePointPriorXY()
  {
  }

  void computeError() override
  {
    const g2o::VertexSE3*v = static_cast<const g2o::VertexSE3 *>(_vertices[0]);

    Eigen::Vector3d estimated = v->estimate().translation();
    _error[0] = estimated[0] - _measurement[0];
    _error[1] = estimated[1] - _measurement[1];
  }

  void setMeasurement(const Eigen::Vector2d &m) override
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
