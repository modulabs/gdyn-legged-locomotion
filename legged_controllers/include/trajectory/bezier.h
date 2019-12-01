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
  Bezier()
  {
    _n_order = N_PNT-1;
    for (size_t i=0; i<=N_PNT; i++)
      _b[i] = boost::math::binomial_coefficient<double>(_n_order, i);
  }

  typedef Eigen::Matrix<double, N_DIM, 1> VectorNd;

  void setPoints(const array<VectorNd, N_PNT>& pnts) {_pnts = pnts;}

  const VectorNd& getPoint(double t)
  {
    _p.setZero();
    for(size_t i=0; i<=N_PNT; i++)
      _p +=  _b[i] * pow(1-t, _n_order) * pow(t, i) * _pnts[i];

    return _p;
  }

private:
  size_t                  _n_order;
  array<VectorNd, N_PNT>  _pnts;
  array<double,N_PNT>     _b;       // coefficient
  VectorNd                _p;

};

}
