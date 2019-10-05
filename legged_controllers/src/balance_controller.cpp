#include <legged_controllers/balance_controller.h>

void BalanceController::init(double m_body, const Eigen::Vector3d& p_body2com, const Eigen::Matrix3d& I_com, double mu)
{
    _m_body = m_body;
    _p_body2com = p_body2com;
    _I_com = I_com;
    _mu = mu;
}

void BalanceController::setControlInput(const Eigen::Vector3d& p_body_d, 
            const Eigen::Vector3d& p_body_dot_d, 
            const Eigen::Matrix3d& R_body_d, 
            const Eigen::Vector3d& w_body_d,
			const Eigen::Vector3d& p_body, 
            const Eigen::Vector3d& p_body_dot,
            const Eigen::Matrix3d& R_body,
            const Eigen::Vector3d w_body,
            const std::array<Eigen::Vector3d,4>& p_body2leg)
{
    // to world coordinates
    for (int i=0; i<4; i++)
        _p_leg[i] = p_body + R_body * p_body2leg[i];

    // rotation
    _w_body_d = w_body_d;
    _w_body = w_body;

    _R_body_d = R_body_d;
    _R_body = R_body;

    // body to com
    _p_com_d = p_body_d + R_body_d * _p_body2com;
    _p_com = p_body + R_body * _p_body2com;
    _p_com_dot_d = p_body_dot_d + skew(w_body_d) * R_body_d * _p_body2com;
}

void BalanceController::getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg)
{
    F_leg[0] = _F.segment(0,3);
    F_leg[1] = _F.segment(3, 3);
    F_leg[2] = _F.segment(6, 3);
    F_leg[3] = _F.segment(9, 3);
}

void BalanceController::update()
{
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


    bd.head(3) = _m_body * ( _kp_p.cwiseProduct(_p_com_d - _p_com) + _kd_p.cwiseProduct(_p_com_dot_d - _p_com_dot) + Eigen::Vector3d(0,0,GRAVITY_CONSTANT) );
    bd.tail(3) = _R_body*_I_com*_R_body.transpose() * ( _kp_w.cwiseProduct(logR(_R_body_d*_R_body.transpose())) + _kd_w.cwiseProduct(_w_body_d - _w_body) );

    for (int i=0; i<4; i++)
    {
        A.block<3,3>(0,3*i) = Eigen::Matrix3d::Identity();
        A.block<3,3>(3,3*i) = skew(_p_leg[i] - _p_com);
    }
    
    H.topLeftCorner(12, 12) = A.transpose()*S*A + Alpha + Beta;

    g = -A.transpose()*S*bd - beta*F_prev;

    static int td = 0;
    // if (td==0)
    // {
    //     std::cout << "A = " << A << std::endl;
    //     std::cout << "H = " << H << std::endl;
    //     std::cout << "g = " << g << std::endl;
    //     std::cout << "bd = " << bd << std::endl;
    // }
    if (td++==1000)
    {
        td = 0;
        printf("p_com_d = %f, %f, %f", _p_com_d(0), _p_com_d(1), _p_com_d(2));
        printf("p_com = %f, %f, %f", _p_com(0), _p_com(1), _p_com(2));

        printf("bd = %f, %f, %f\n", bd(0), bd(1), bd(2));

    }

    // Inequality constraint matrix from friction cone
    for (int i=0; i<4; i++)
    {
        C.block<4,3>(4*i,3*i) << 1, 0, -_mu,    
                                -1, 0, -_mu,
                                 0, 1, -_mu,
                                0, -1, -_mu;
    }

    // Optimization
    USING_NAMESPACE_QPOASES

    real_t fz_max = 400;    // 600N is total mass load of hyq, later have to get this value from actuator capacity
    real_t lb[12] = {-_mu*fz_max, -_mu*fz_max, 10,
                    -_mu*fz_max, -_mu*fz_max, 10,
                    -_mu*fz_max, -_mu*fz_max, 10,
                    -_mu*fz_max, -_mu*fz_max, 10};
    real_t ub[12] = {_mu*fz_max, _mu*fz_max, fz_max,
                    _mu*fz_max, _mu*fz_max, fz_max,
                    _mu*fz_max, _mu*fz_max,fz_max,
                    _mu*fz_max, _mu*fz_max, fz_max};
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

    _F << xOpt[0], xOpt[1], xOpt[2],
        xOpt[3], xOpt[4], xOpt[5],
        xOpt[6], xOpt[7], xOpt[8],
        xOpt[9], xOpt[10], xOpt[11];

    F_prev = _F;

    // if (td==0)
    // {
    //     std::cout << "F = " << _F << std::endl;
    // }
    // td++;

    _F.segment(0,3) = -_R_body.transpose()*_F.segment(0,3);
    _F.segment(3,3) = -_R_body.transpose()*_F.segment(3,3);
    _F.segment(6,3) = -_R_body.transpose()*_F.segment(6,3);
    _F.segment(9,3) = -_R_body.transpose()*_F.segment(9,3);

    
}