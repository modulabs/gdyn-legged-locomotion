#include <legged_controllers/balance_controller.h>

void BalanceController::init()
{
}

//void BalanceController::setControlInput(const Eigen::Vector3d& p_body_d,
//            const Eigen::Vector3d& p_body_dot_d,
//            const Eigen::Matrix3d& R_body_d,
//            const Eigen::Vector3d& w_body_d,
//			const Eigen::Vector3d& p_body,
//            const Eigen::Vector3d& p_body_dot,
//            const Eigen::Matrix3d& R_body,
//            const Eigen::Vector3d w_body,
//            const std::array<Eigen::Vector3d,4>& p_body2leg)
//{
//    // to world coordinates
//    for (int i=0; i<4; i++)
//        _p_leg[i] = p_body + R_body * p_body2leg[i];

//    // rotation
//    _w_body_d = w_body_d;
//    _w_body = w_body;

//    _R_body_d = R_body_d;
//    _R_body = R_body;

//    // body to com
//    _p_com_d = p_body_d + R_body_d * _p_body2com;
//    _p_com = p_body + R_body * _p_body2com;
//    _p_com_dot_d = p_body_dot_d + skew(w_body_d) * R_body_d * _p_body2com;
//}

//void BalanceController::getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg)
//{
//    F_leg[0] = _F.segment(0,3);
//    F_leg[1] = _F.segment(3, 3);
//    F_leg[2] = _F.segment(6, 3);
//    F_leg[3] = _F.segment(9, 3);
//}

void BalanceController::update(quadruped_robot::QuadrupedRobot& robot, std::array<Eigen::Vector3d, 4>& F_leg)
{
  // input
  double m = robot._m_body;
  double mu = robot._mu_foot;
  Eigen::Matrix3d& I_com = robot._I_com_body;
  Eigen::Vector3d& p_com_d = robot._pose_com_d._pos;
  Eigen::Vector3d& p_com = robot._pose_com._pos;
  Eigen::Vector3d& v_com_d = robot._pose_vel_com_d._linear;
  Eigen::Vector3d& v_com = robot._pose_vel_com._linear;
  Eigen::Matrix3d R_d = robot._pose_body_d._rot_quat.toRotationMatrix();
  Eigen::Matrix3d R = robot._pose_body._rot_quat.toRotationMatrix();
  Eigen::Vector3d& w_d = robot._pose_vel_body_d._angular;
  Eigen::Vector3d& w = robot._pose_vel_body._angular;
  const std::array<Eigen::Vector3d, 4>& p_leg = robot._p_world2leg;

    // Optimization
    Eigen::Matrix<double, 6, 12> A;
    Eigen::Matrix<double, 12, 1> F;
    static Eigen::Matrix<double, 12, 1> F_prev;
    Eigen::Matrix<double, 6, 1> bd;
    Eigen::Matrix<double, 6, 6> S;
    double alpha=0.1;
    double beta=0.1;

    // Transform from raw optimizaiton form to QP optimizer form
    Eigen::Matrix<double, 12, 12, Eigen::RowMajor> H = Eigen::Matrix<double, 12, 12, Eigen::RowMajor>::Zero();
    Eigen::Matrix<double, 12, 12> Alpha = alpha * Eigen::Matrix<double, 12,12>::Identity();
    Eigen::Matrix<double, 12, 12> Beta = beta * Eigen::Matrix<double, 12,12>::Identity();
    Eigen::Matrix<double, 12, 1> g;
    Eigen::Matrix<double, 16, 12, Eigen::RowMajor> C = Eigen::Matrix<double, 16, 12, Eigen::RowMajor>::Zero();
 
    // 
    _kp_p << 50, 50, 100;
    _kd_p << 10, 10, 20;
    _kp_w << 100, 100, 100;
    _kd_w << 20, 20, 20;

    S.setZero();
    S.diagonal() << 1, 1, 1, 2, 2, 2;

    bd.head(3) = m * ( _kp_p.cwiseProduct(p_com_d - p_com) + _kd_p.cwiseProduct(v_com_d - v_com) + Eigen::Vector3d(0,0,GRAVITY_CONSTANT) );
    bd.tail(3) = R*I_com*R.transpose() * ( _kp_w.cwiseProduct(logR(R_d*R.transpose())) + _kd_w.cwiseProduct(w_d - w) );

    for (int i=0; i<4; i++)
    {
        A.block<3,3>(0,3*i) = Eigen::Matrix3d::Identity();
        A.block<3,3>(3,3*i) = skew(p_leg[i] - p_com);
    }
    
    H.topLeftCorner(12, 12) = A.transpose()*S*A + Alpha + Beta;

    g = -A.transpose()*S*bd - beta*F_prev;

    // Inequality constraint matrix from friction cone
    for (int i=0; i<4; i++)
    {
        C.block<4,3>(4*i,3*i) << 1, 0, -mu,
                                -1, 0, -mu,
                                 0, 1, -mu,
                                0, -1, -mu;
    }

    // Optimization
    USING_NAMESPACE_QPOASES

    real_t fz_max = 400;    // 600N is total mass load of hyq, later have to get this value from actuator capacity
    real_t lb[12] = {-mu*fz_max, -mu*fz_max, 10,
                    -mu*fz_max, -mu*fz_max, 10,
                    -mu*fz_max, -mu*fz_max, 10,
                    -mu*fz_max, -mu*fz_max, 10};
    real_t ub[12] = {mu*fz_max, mu*fz_max, fz_max,
                    mu*fz_max, mu*fz_max, fz_max,
                    mu*fz_max, mu*fz_max,fz_max,
                    mu*fz_max, mu*fz_max, fz_max};
    real_t ubA[16] = {0.0,};

    QProblem qp_problem( 12, 16);
    // QProblemB qp_problem( 12 );

    Options options;
    qp_problem.setOptions( options );

    int nWSR = 1000;
    qp_problem.init(H.data(), g.data(), C.data(), lb, ub, NULL, ubA, nWSR);
    // qp_problem.init(H.data(), g.data(), lb, ub, nWSR);

    real_t xOpt[12];
    real_t yOpt[12+1];
    qp_problem.getPrimalSolution( xOpt );

    F << xOpt[0], xOpt[1], xOpt[2],
        xOpt[3], xOpt[4], xOpt[5],
        xOpt[6], xOpt[7], xOpt[8],
        xOpt[9], xOpt[10], xOpt[11];

    F_prev = F;

    // if (td==0)
    // {
    //     std::cout << "F = " << _F << std::endl;
    // }
    // td++;

    for (int i=0; i<4; i++)
      F_leg[i] = -R.transpose()*F.segment(3*i,3*(i+1));
}
