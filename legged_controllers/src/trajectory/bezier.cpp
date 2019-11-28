#include <trajectory/bezier.h>

namespace trajectory
{


template<size_t N_DIM, size_t N_PNT>
Bezier<N_DIM, N_PNT>::Bezier()
{
  _n_order = N_PNT-1;
  for (size_t i=0; i<=N_PNT; i++)
    _b[i] = boost::math::binomial_coefficient<double>(_n_order, i);
}


//template<size_t N_DIM, size_t N_PNT>
//const VectorNd& Bezier::getPoint(double t)
//{
//  _p.setZero();
//  for(size_t i=0; i<=N_PNT; i++)
//    _p +=  _b[i] * pow(1-t, _n_order) * pow(t, i) * _pnts[i];

//  return _p;
//}



}
