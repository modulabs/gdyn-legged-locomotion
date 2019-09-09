#include <legged_controllers/mpc_controller.h>

void MPCController::init(double m_body, const Eigen::Vector3d& p_body2com, const Eigen::Matrix3d& I_com, double mu)
{
    _m_body = m_body;
    _p_body2com = p_body2com;
    _I_com = I_com;
    _mu = mu;
}

void MPCController::setControlInput(const Eigen::Vector3d& p_body_d, 
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

void MPCController::getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg)
{
    F_leg[0] = _F.segment(0,3);
    F_leg[1] = _F.segment(3, 3);
    F_leg[2] = _F.segment(6, 3);
    F_leg[3] = _F.segment(9, 3);
}

void MPCController::update()
{
    // Not tested yet
  
    // Continuous Simplified Robot Dynamics x_dot = A_c*x + B_c*u

    Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(15,15);
    Eigen::MatrixXd A_d = Eigen::MatrixXd::Zero(15,15);
    Eigen::MatrixXd B_c = Eigen::MatrixXd::Zero(15,15);
    Eigen::MatrixXd B_d = Eigen::MatrixXd::Zero(15,15);

    Eigen::MatrixXd I3x3 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd I15x15 = Eigen::MatrixXd::Identity(15,15);

    Eigen::MatrixXd I_hat = Eigen::MatrixXd::Zero(3,3);
    
    Eigen::MatrixXd p_leg_1_skew = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd p_leg_2_skew = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd p_leg_3_skew = Eigen::MatrixXd::Zero(3,3);
    Eigen::MatrixXd p_leg_4_skew = Eigen::MatrixXd::Zero(3,3);

    Eigen::MatrixXd T15X15 = Eigen::MatrixXd::Identity(15,15);

    Eigen::Vector3d EulerAngle = _R_body_d.eulerAngles(2, 1, 0); 
    Eigen::Matrix3d Rz;
    Rz = AngleAxisd(_R_body_d(2), Vector3d::UnitZ());

    A_c.block<3,3>(0,6) = Rz;
    A_c.block<3,3>(3,9) = I3x3;
    A_c.block<3,3>(9,12) = -I3x3;

    I_hat = _I_com;
  
    I_hat = Rz*I_hat*Rz.transpose();

    p_leg_1_skew = skew(_p_leg[0]);
    p_leg_2_skew = skew(_p_leg[1]);
    p_leg_3_skew = skew(_p_leg[2]);
    p_leg_4_skew = skew(_p_leg[3]);

    B_c.block<3,3>(6,0) = I_hat.inverse()*p_leg_1_skew;
    B_c.block<3,3>(6,3) = I_hat.inverse()*p_leg_2_skew;
    B_c.block<3,3>(6,6) = I_hat.inverse()*p_leg_3_skew;
    B_c.block<3,3>(6,9) = I_hat.inverse()*p_leg_4_skew;

    B_c.block<3,3>(9,0) = (1/_m_body)*I3x3;
    B_c.block<3,3>(9,3) = (1/_m_body)*I3x3;
    B_c.block<3,3>(9,6) = (1/_m_body)*I3x3;
    B_c.block<3,3>(9,9) = (1/_m_body)*I3x3;

    // Discrete Simplified Robot Dynamics x[k+1] = A_c*x[k] + B_c*u[k]

    A_d = I15x15+A_c*(SamplingTime*T15X15);
    B_d = B_c*(SamplingTime*T15X15);

    // Condensed QP formulation X = Aqp*x0 + Bqp*U

    Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(15*(MPC_Step+1), 15);
    A_qp.block<15,15>(0,0) = I15x15;

    Eigen::MatrixXd Temp = Eigen::MatrixXd::Zero(15,15);
    Temp = A_d;

    for(int i=1; i<=MPC_Step; i++)
    {
        A_qp.block<15,15>(15*i,0) = Temp;
        Temp *= A_d; 
    }

    Temp = B_d;
    Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero(15*(MPC_Step+1), 15*MPC_Step);

    for(int i=0; i<MPC_Step; i++)
    {
        Temp = B_d;    

        for(int k=0; k<MPC_Step; k++)
        {
            for(int j=(1+i); j<=MPC_Step; j++)
            {
                B_qp.block<15,15>(15*j,k) = Temp;

                Temp = A_d*B_d;
            }
        }   
    }

    Eigen::MatrixXd L_d = Eigen::MatrixXd::Identity(15,15);
    Eigen::MatrixXd K_d = Eigen::MatrixXd::Identity(15,15);

    L_d(0,0) = L_00_gain*L_d(0,0);
    L_d(1,1) = L_11_gain*L_d(1,1);
    L_d(2,2) = L_22_gain*L_d(2,2);
    L_d(3,3) = L_33_gain*L_d(3,3);
    L_d(4,4) = L_44_gain*L_d(4,4);

    K_d(0,0) = K_00_gain*L_d(0,0);
    K_d(1,1) = K_11_gain*L_d(1,1);
    K_d(2,2) = K_22_gain*L_d(2,2);
    K_d(3,3) = K_33_gain*L_d(3,3);
    K_d(4,4) = K_44_gain*L_d(4,4);    

    Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(15*MPC_Step, 15*MPC_Step);
    Eigen::MatrixXd L_qp = Eigen::MatrixXd::Zero(15*(MPC_Step+1), 15*(MPC_Step+1));
    Eigen::MatrixXd K_qp = Eigen::MatrixXd::Zero(15*MPC_Step, 15*MPC_Step);

    for(int i=0; i<(MPC_Step+1); i++)
        L_qp.block<15,15>(15*i,15*i) = L_d;

    for(int i=0; i<MPC_Step; i++)
        K_qp.block<15,15>(15*i,15*i) = K_d;        

    H_qp = B_qp.transpose()*L_qp*B_qp + K_qp;
    
    Eigen::MatrixXd g_qp = Eigen::MatrixXd::Zero(15*MPC_Step, 1);
    Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(15, 1);

    x0(0) = 0.0;
    x0(1) = 0.0;
    x0(2) = EulerAngle(2);
    x0(3) = _p_com(0);
    x0(4) = _p_com(1);
    x0(5) = _p_com(2);
    x0(6) = _w_body(0);
    x0(7) = _w_body(1);
    x0(8) = _w_body(2);
    x0(9) = _p_com_dot(0);
    x0(10) = _p_com_dot(1);
    x0(11) = _p_com_dot(2);
    x0(12) = 0.0;
    x0(13) = 0.0;
    x0(14) = -9.8;

    g_qp = B_qp.transpose()*L_qp*A_qp*x0;

    // Inequality constraint

    Eigen::MatrixXd C_1leg = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd C_4leg = Eigen::MatrixXd::Zero(6*4,3);
    Eigen::MatrixXd C_qp = Eigen::MatrixXd::Zero(6*4*MPC_Step,3*MPC_Step);

    C_1leg <<  0,  0,   -1,
               0,  0,    1,
              -1,  0, -_mu,
               1,  0, -_mu,
               0, -1, -_mu,
               0,  1, -_mu;

    for(int i=0; i<4; i++)
        C_4leg.block<6,3>(6*i,0) = C_1leg;  

    for(int i=0; i<MPC_Step; i++)
        C_qp.block<6*4,3>(6*4*i,3*i) = C_4leg;

    Eigen::MatrixXd lbC_1leg = Eigen::MatrixXd::Zero(6,1);
    Eigen::MatrixXd lbC_4leg = Eigen::MatrixXd::Zero(6*4,1);
    Eigen::MatrixXd lbC_qp = Eigen::MatrixXd::Zero(6*4*MPC_Step,1*MPC_Step);

    lbC_1leg << -Force_min,
                 Force_max,
                         0,
                         0,
                         0,
                         0;

    for(int i=0; i<4; i++)
        lbC_4leg.block<6,1>(6*i,1*0) = lbC_1leg;     

    for(int i=0; i<MPC_Step; i++)
        lbC_qp.block<6*4,1>(6*4*i,1*i) = lbC_4leg;                    

    // Optimization(QP Solver)

    USING_NAMESPACE_QPOASES

    real_t H_qp_qpoases[(15*MPC_Step)*(15*MPC_Step)] = {0, };
    real_t g_qp_qpoases[(15*MPC_Step)*(1)] = {0, };
    real_t C_qp_qpoases[(6*4*MPC_Step)*(3*MPC_Step)] = {0, };
    real_t lbC_qp_qpoases[(6*4*MPC_Step)*(1*MPC_Step)] = {0, };

    for(int i=0; i<(15*MPC_Step)*(15*MPC_Step); i++)
        H_qp_qpoases[i] = H_qp(i);

    for(int i=0; i<(15*MPC_Step)*(1); i++)
        g_qp_qpoases[i] = g_qp(i);

    for(int i=0; i<(6*4*MPC_Step)*(3*MPC_Step); i++)
        C_qp_qpoases[i] = C_qp(i);

    for(int i=0; i<(6*4*MPC_Step)*(1*MPC_Step); i++)
        lbC_qp_qpoases[i] = lbC_qp(i);             
    
    QProblem qp_problem( 12*MPC_Step,1 );

    Options options;
	qp_problem.setOptions( options );

    int nWSR = 10;
    qp_problem.init(H_qp_qpoases, g_qp_qpoases, C_qp_qpoases, NULL, NULL, lbC_qp_qpoases, NULL, nWSR);

	real_t UOpt[12*MPC_Step];
    qp_problem.getPrimalSolution( UOpt );

    for(int i=0; i<12; i++)
        _F[i] = UOpt[i];       
}