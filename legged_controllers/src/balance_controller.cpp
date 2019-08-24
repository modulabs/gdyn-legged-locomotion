#include <legged_controllers/balance_controller.h>

void BalanceController::init(double m_body, const KDL::RotationalInertia& I_com, double mu)
{
    _m_body = m_body;
    _I_com = I_com;
    _mu = mu;
}

void BalanceController::setControlInput(const KDL::Vector& p_com_d, 
            const KDL::Vector& p_com_dot_d, 
            const KDL::Rotation& R_body_d, 
            const KDL::Vector& w_body_d,
			const KDL::Vector& p_com, 
            const std::array<KDL::Vector,4>& p_leg,
            const KDL::Rotation& R_body,
            const KDL::Vector w_body)
{
    _p_com_d = p_com;
    _p_com_dot_d = p_com_dot_d;
    _R_body_d = R_body_d;
    _w_body_d = w_body_d;
    _p_com = p_com;
    _p_leg = p_leg;
    _R_body = R_body;
    _w_body = w_body;
}

void BalanceController::getControlOutput(std::array<KDL::Vector, 4>& F_leg)
{
    _F_leg = F_leg;
}

void BalanceController::update()
{
    // Not tested yet
    
    // Dynamics
    Eigen::Matrix<double, 6, 12> A;
    Eigen::Matrix<double, 12, 1> F;
    static Eigen::Matrix<double, 12, 1> F_prev;
    Eigen::Matrix<double, 6, 1> bd;
    Eigen::Matrix<double, 6, 6> S;
    double alpha;
    double beta;

    // Transform to QP optimizer form
    Eigen::Matrix<double, 14, 14, Eigen::RowMajor> H = Eigen::Matrix<double, 14, 14, Eigen::RowMajor>::Zero();
    Eigen::Matrix<double, 12, 1> g;
    Eigen::Matrix<double, 8, 3, Eigen::RowMajor> C = Eigen::Matrix<double, 8, 3, Eigen::RowMajor>::Zero();

    H.topLeftCorner(12, 12) = A.transpose()*S*A;
    H(12, 12) = alpha;
    H(13, 13) = beta;

    g = -A.transpose()*S*bd - beta*F_prev;

    C << 1, 0, -_mu,
        0, 1, -_mu,
        1, 0, -_mu,
        0, 1, -_mu,
        1, 0, -_mu,
        0, 1, -_mu,
        1, 0, -_mu,
        0, 1, -_mu;

    // Optimization
    USING_NAMESPACE_QPOASES

    real_t A_[8*3];
    real_t lb[12];
    real_t ub[12];
    real_t lbA[8];

    QProblem qp_problem( 12,1 );

    Options options;
	qp_problem.setOptions( options );

    int nWSR = 10;
    qp_problem.init(H.data(), g.data(), C.data(), lb, ub, lbA, NULL, nWSR);

	real_t xOpt[12];
	real_t yOpt[12+1];
    qp_problem.getPrimalSolution( xOpt );
}

