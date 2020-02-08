/*
  Author: Modulabs
  File Name: balance_controller.cpp
*/

#include "legged_robot_controller/balance_controller.h"


void BalanceController::init()
{
  _legs.reserve(4);
}

void BalanceController::update(quadruped_robot::QuadrupedRobot& robot, std::array<Vector3d, 4>& F_leg)
{
  // input
  double m = robot._m_body;
  double mu = robot._mu_foot;
  const Matrix3d& I_com = robot._I_com_body;
  const Vector3d& p_com_d = robot._pose_com_d._pos;
  const Vector3d& p_com = robot._pose_com._pos;
  const Vector3d& v_com_d = robot._pose_vel_com_d._linear;
  const Vector3d& v_com = robot._pose_vel_com._linear;
  const Matrix3d R_d = robot._pose_body_d._rot_quat.toRotationMatrix();
  const Matrix3d R = robot._pose_body._rot_quat.toRotationMatrix();
  const Vector3d& w_d = robot._pose_vel_body_d._angular;
  const Vector3d& w = robot._pose_vel_body._angular;
  const std::array<Vector3d, 4>& p_leg = robot._p_world2leg;
  const std::array<int, 4>& contact_states = robot._contact_states;
  const std::array<Vector3d, 4>& F_total_prev = robot._F_world2leg_prev;

  // contact number
  _legs.clear();
  for (size_t i=0; i<4; i++)
  {
    if (robot.getController(i) == quadruped_robot::controllers::BalancingQP && contact_states[i] == 1)
      _legs.push_back(i);
  }

  if (_legs.size() < 1)
    return;

  int opt_size = 3*_legs.size();
  int inequality_constraint_size = 4*_legs.size();

//  // Optimization
//  Matrix<double, 6, Dynamic, ColMajor, 6, 12> A;
//  Matrix<double, Dynamic, 1, ColMajor, 12, 1> F;
//  Matrix<double, Dynamic, 1, ColMajor, 12, 1> F_prev;
//  Matrix<double, 6, 1> bd;
//  Matrix<double, 6, 6> S;
//  double alpha=0.01;
//  double beta=0.01;

//  // QP optimization
//  Matrix<double, Dynamic, Dynamic, RowMajor, 12, 12> H;
//  Matrix<double, Dynamic, Dynamic, ColMajor, 12, 12> Alpha;
//  Matrix<double, Dynamic, Dynamic, ColMajor, 12, 12> Beta;
//  Matrix<double, Dynamic, 1, ColMajor, 12, 1> g;

//  // Inequality constraint
//  double fz_max = 400;    // 600N is total mass load of hyq, later have to get this value from actuator capacity
//  Matrix<double, Dynamic, Dynamic, RowMajor, 16, 12> C;
//  Matrix<double, 1, Dynamic, RowMajor, 1, 12> lb, ub;
//  Matrix<double, 1, Dynamic, RowMajor, 1, 16> ubC;

  // Resize
  _A.resize(NoChange, opt_size);
  _F.resize(opt_size);
  _F_prev.resize(opt_size);
  _H.resize(opt_size, opt_size);
  _Alpha.resize(opt_size, opt_size);
  _Beta.resize(opt_size, opt_size);
  _g.resize(opt_size);
  _C.resize(inequality_constraint_size, opt_size);
  _lb.resize(opt_size);
  _ub.resizeLike(_lb);
  _ubC.resize(inequality_constraint_size);

  //
  _H.setZero();
  _C.setZero();
  _ubC.setZero();

  // gains
  _kp_p << 100, 200, 100;
  _kd_p << 20, 60, 20;
  _kp_w << 1200, 800, 400;
  _kd_w << 120, 80, 100;

  _S.setZero();
  _S.diagonal() << 1, 1, 1, 2, 2, 2;

  double alpha = 0.01;
  double beta = 0.01;
  _Alpha.setIdentity().diagonal().setConstant(alpha);
  _Beta.setIdentity().diagonal().setConstant(beta);

  double fz_max = 400;    // 600N is total mass load of hyq, later have to get this value from actuator capacity

  // Desired acceleration
  _bd.head(3) = m * ( _kp_p.cwiseProduct(p_com_d - p_com) + _kd_p.cwiseProduct(v_com_d - v_com) + Vector3d(0,0,GRAVITY_CONSTANT) );
  _bd.tail(3) = R*I_com*R.transpose() * ( _kp_w.cwiseProduct(logR(R_d*R.transpose())) + _kd_w.cwiseProduct(w_d - w) );

  // Stacking matrix with contact foot variable
  for (int l=0, i=0; l < _legs.size(); l++, i++)
  {
    _F_prev.segment<3>(3*i) = F_total_prev[_legs[l]];

    // Dynamics
    _A.block<3,3>(0,3*i) = Matrix3d::Identity();
    _A.block<3,3>(3,3*i) = skew(p_leg[_legs[l]] - p_com);

    // Inequality constraint matrix from friction cone
    _C.block<4,3>(4*i,3*i) << 1, 0, -mu,
                            -1, 0, -mu,
                             0, 1, -mu,
                            0, -1, -mu;
    _lb.segment<3>(3*i) << -mu*fz_max, -mu*fz_max, 10;
    _ub.segment<3>(3*i) << mu*fz_max, mu*fz_max, fz_max;
  }

  // Objective matrix
  _H = _A.transpose()*_S*_A + _Alpha + _Beta;   // Hessian
  _g = -_A.transpose()*_S*_bd - beta*_F_prev;  // Gradient

  // Optimization
  USING_NAMESPACE_QPOASES

//  real_t lb[12] = {-mu*fz_max, -mu*fz_max, 10,
//                  -mu*fz_max, -mu*fz_max, 10,
//                  -mu*fz_max, -mu*fz_max, 10,
//                  -mu*fz_max, -mu*fz_max, 10};
//  real_t ub[12] = {mu*fz_max, mu*fz_max, fz_max,
//                  mu*fz_max, mu*fz_max, fz_max,
//                  mu*fz_max, mu*fz_max,fz_max,
//                  mu*fz_max, mu*fz_max, fz_max};
//  real_t ubA[16] = {0.0,};

  QProblem qp_problem( opt_size, inequality_constraint_size);

  Options options;
  qp_problem.setOptions( options );

  int nWSR = 100;
  qp_problem.init(_H.data(), _g.data(), _C.data(), _lb.data(), _ub.data(), NULL, _ubC.data(), nWSR);

  real_t xOpt[12];
//  real_t yOpt[12+1];
  qp_problem.getPrimalSolution( xOpt );

  for (int i=0; i<opt_size; i++)
  {
    _F(i) = xOpt[i];
  }

  for (int l=0, i=0; l<_legs.size(); l++, i++)
  {
    F_leg[_legs[l]] = -R.transpose()*_F.segment<3>(3*i);
  }
}
