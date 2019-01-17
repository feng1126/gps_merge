#ifndef G2O_TARGET_TYPES_3D_HPP_
#define G2O_TARGET_TYPES_3D_HPP_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
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

// This header file specifies a set of types for the different
// tracking examples; note that
namespace g2o
{

class VertexPosition3D : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPosition3D()
  {
  }

  virtual void setToOriginImpl()
  {
    _estimate.setZero();
  }

  virtual void oplusImpl(const double *update)
  {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
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

// The idealised GPS measurement; this is 3D and linear
class GPSObservationPosition3DEdge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPosition3D>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GPSObservationPosition3DEdge()
  {
  }

  void computeError()
  {
    const VertexPosition3D *v = static_cast<const VertexPosition3D *>(_vertices[0]);
    _error = v->estimate() - _measurement;
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




// class EdgeDist3D : public BaseBinaryEdge<1, double,  VertexPosition3D, VertexPosition3D>
// {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

//   EdgeDist3D()
//   {
//   }

//   void computeError() override
//   {
  
//    const VertexPosition3D *v1 = static_cast<const VertexPosition3D *>(_vertices[0]);
//    const VertexPosition3D *v2 = static_cast<const VertexPosition3D *>(_vertices[1]);
//    Eigen::Vector3d p = v1->estimate();

//     // double estim_dist = ;

//     _error[0]= estim_dist - _measurement;
//   }

//   void setMeasurement(const double &m) override
//   {
//     _measurement = m;
//   }
//   virtual bool read(std::istream & /*is*/)
//   {
//     return false;
//   }

//   virtual bool write(std::ostream & /*os*/) const
//   {
//     return false;
//   }
// }; // class EdgeSE3ExpmapPrior3D





// class errorRT3D : public BaseUnaryEdge<1, double,  VertexPosition3D>
// {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

//   errorRT3D()
//   {
//   }

//   void computeError() override
//   {
  
//    const VertexPosition3D *v1 = static_cast<const VertexPosition3D *>(_vertices[0]);
//    const VertexPosition3D *v2 = static_cast<const VertexPosition3D *>(_vertices[1]);
//     Eigen::Vector3d p1 = v1->estimate();
//     Eigen::Vector3d p2 = v2->estimate();
//     Eigen::Vector3d dist_ = p2 - p1;

//     double estim_dist = sqrt(pow(dist_[0], 2) + pow(dist_[1], 2) + pow(dist_[2], 2));

//     _error[0]= estim_dist - _measurement;
//   }

//   void setMeasurement(const double &m) override
//   {
//     _measurement = m;
//   }
//   virtual bool read(std::istream & /*is*/)
//   {
//     return false;
//   }

//   virtual bool write(std::ostream & /*os*/) const
//   {
//     return false;
//   }
// }; // class EdgeSE3ExpmapPrior3D

// Store velocity separately from position?
class VertexVelocity3D : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexVelocity3D()
  {
  }

  virtual void setToOriginImpl()
  {
    _estimate.setZero();
  }

  virtual void oplusImpl(const double *update)
  {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
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

class EdgePosition3Dist : public g2o::BaseUnaryEdge<1, double, VertexPosition3D>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgePosition3Dist()
  {
  }

  void computeError() override
  {

    const VertexPosition3D *v1 = static_cast<const VertexPosition3D *>(_vertices[0]);
    const VertexPosition3D *v2 = static_cast<const VertexPosition3D *>(_vertices[1]);
    Eigen::Vector3d p1 = v1->estimate();
    Eigen::Vector3d p2 = v2->estimate();
    Eigen::Vector3d dist_ = p2 - p1;

    double estim_dist = sqrt(pow(dist_[0], 2) + pow(dist_[1], 2) + pow(dist_[2], 2));

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
}; //

} // namespace g2o

#endif //  __TARGET_TYPES_3D_HPP__
