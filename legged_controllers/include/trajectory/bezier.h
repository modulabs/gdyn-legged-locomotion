#pragma once

#include <Eigen/Core>
#include <boost/math/special_functions/binomial.hpp>

using Eigen::Matrix;
using std::array;

namespace trajectory
{


template<size_t N_DIM, size_t N_PNT>
class Bezier
{


public:
  Bezier();

  typedef Eigen::Matrix<double, N_DIM, 1> VectorNd;

  void setPoints(const array<VectorNd, N_PNT>& pnts) {_pnts = pnts;}

  const VectorNd& getPoint(double t);

private:
  size_t                  _n_order;
  array<VectorNd, N_PNT>  _pnts;
  array<double,N_PNT>     _b;       // coefficient
  VectorNd                _p;

};

}
