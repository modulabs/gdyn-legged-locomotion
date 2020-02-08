/*
  Author: Modulabs
  File Name: bezier.h
*/

#pragma once

#include <Eigen/Core>
#include <boost/math/special_functions/binomial.hpp>

using Eigen::Matrix;
using Eigen::Vector2d;
using std::array;
using std::vector;

namespace trajectory
{
#if 0
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

#else

class Bezier
{


public:
  Bezier()
  {
  }

  void setPoints(const vector<Vector2d>& pnts)
  {
    size_t n_size = pnts.size();
    _n_order = n_size-1;
    _pnts = pnts;
    _b.resize(n_size);
    for (size_t i=0; i<=_n_order; i++)
      _b[i] = boost::math::binomial_coefficient<double>(_n_order, i);
  }

  const Vector2d& getPoint(double t)
  {
    _p.setZero();
    for(size_t i=0; i<=_n_order; i++)
      _p +=  _b[i] * pow(1-t, _n_order-i) * pow(t, i) * _pnts[i];

    return _p;
  }

public:
  size_t                  _n_order;
  vector<Vector2d>        _pnts;
  vector<double>          _b;       // coefficient
  Vector2d                _p;

};
#endif
}
