#include <legged_controllers/mpc_controller.h>

void MPCController::init(double m_body, const Eigen::Vector3d& p_body2com, const Eigen::Matrix3d& I_com, double mu)
{
    _m_body = m_body;
    _p_body2com = p_body2com;
    _I_com = I_com;
    _mu = mu;

    _A_c = Eigen::MatrixXd::Zero(15,15);
    _A_d = Eigen::MatrixXd::Zero(15,15);
    _B_c = Eigen::MatrixXd::Zero(15,12);
    _B_d = Eigen::MatrixXd::Zero(15,12);

    _I3x3 = Eigen::MatrixXd::Identity(3,3);
    _I15x15 = Eigen::MatrixXd::Identity(15,15);

    _I_hat = Eigen::MatrixXd::Zero(3,3);
    
    _p_leg_1_skew = Eigen::MatrixXd::Zero(3,3);
    _p_leg_2_skew = Eigen::MatrixXd::Zero(3,3);
    _p_leg_3_skew = Eigen::MatrixXd::Zero(3,3);
    _p_leg_4_skew = Eigen::MatrixXd::Zero(3,3);

    _T15X15 = Eigen::MatrixXd::Identity(15,15);
    _T12X12 = Eigen::MatrixXd::Identity(12,12);

    _A_qp = Eigen::MatrixXd::Zero(15*(MPC_Step+1), 15);  
    _Temp = Eigen::MatrixXd::Zero(15,15);
    _B_qp = Eigen::MatrixXd::Zero(15*(MPC_Step+1), 12*MPC_Step); 

    _L_d = Eigen::MatrixXd::Identity(15,15);
    _K_d = Eigen::MatrixXd::Identity(12,12); 

    _H_qp = Eigen::MatrixXd::Zero(12*MPC_Step, 12*MPC_Step);
    _L_qp = Eigen::MatrixXd::Zero(15*(MPC_Step+1), 15*(MPC_Step+1));
    _K_qp = Eigen::MatrixXd::Zero(12*MPC_Step, 12*MPC_Step);   

    _g_qp = Eigen::MatrixXd::Zero(12*MPC_Step, 1);
    _x0 = Eigen::MatrixXd::Zero(15, 1);
    _xref = Eigen::MatrixXd::Zero(15, 1);  

    _C_1leg = Eigen::MatrixXd::Zero(6,3);
    _C_4leg = Eigen::MatrixXd::Zero(6*4,3);
    _C_qp = Eigen::MatrixXd::Zero(6*4*MPC_Step,3*MPC_Step);

    _lbC_1leg = Eigen::MatrixXd::Zero(6,1);
    _lbC_4leg = Eigen::MatrixXd::Zero(6*4,1);
    _lbC_qp = Eigen::MatrixXd::Zero(6*4*MPC_Step,1*MPC_Step);                       
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

    Eigen::Vector3d EulerAngle = _R_body_d.eulerAngles(2, 1, 0); 
    Eigen::Matrix3d Rz;
    Rz = AngleAxisd(_R_body_d(2), Vector3d::UnitZ());

    _A_c.block<3,3>(0,6) = Rz;
    _A_c.block<3,3>(3,9) = _I3x3;
    _A_c.block<3,3>(9,12) = -_I3x3;

    _I_hat = _I_com;
  
    _I_hat = Rz*_I_hat*Rz.transpose();

    _p_leg_1_skew = skew(_p_leg[0]);
    _p_leg_2_skew = skew(_p_leg[1]);
    _p_leg_3_skew = skew(_p_leg[2]);
    _p_leg_4_skew = skew(_p_leg[3]);

    _B_c.block<3,3>(6,0) = _I_hat.inverse()*_p_leg_1_skew;
    _B_c.block<3,3>(6,3) = _I_hat.inverse()*_p_leg_2_skew;
    _B_c.block<3,3>(6,6) = _I_hat.inverse()*_p_leg_3_skew;
    _B_c.block<3,3>(6,9) = _I_hat.inverse()*_p_leg_4_skew;

    _B_c.block<3,3>(9,0) = (1/_m_body)*_I3x3;
    _B_c.block<3,3>(9,3) = (1/_m_body)*_I3x3;
    _B_c.block<3,3>(9,6) = (1/_m_body)*_I3x3;
    _B_c.block<3,3>(9,9) = (1/_m_body)*_I3x3;

    // Discrete Simplified Robot Dynamics x[k+1] = A_c*x[k] + B_c*u[k]

    _A_d = _I15x15+_A_c*(SamplingTime*_T15X15);
    _B_d = _B_c*(SamplingTime*_T12X12);

    // Condensed QP formulation X = Aqp*x0 + Bqp*U

    _A_qp.block<15,15>(0,0) = _I15x15;

    _Temp = _A_d;

    for(int i=1; i<=MPC_Step; i++)
    {
        _A_qp.block<15,15>(15*i,0) = _Temp;
        _Temp *= _A_d; 
    }

    _Temp = _B_d;

    for(int i=0; i<MPC_Step; i++)
    {
        _Temp = _B_d;    

        for(int k=0; k<MPC_Step; k++)
        {
            for(int j=(1+i); j<=MPC_Step; j++)
            {
                _B_qp.block<15,12>(15*j,k) = _Temp;

                _Temp = _A_d*_B_d;
            }
        }   
    }

    _L_d.block<3,3>(0,0) = L_00_gain*_I3x3;
    _L_d.block<3,3>(3,3) = L_11_gain*_I3x3;
    _L_d.block<3,3>(6,6) = L_22_gain*_I3x3;
    _L_d.block<3,3>(9,9) = L_33_gain*_I3x3;
    _L_d.block<3,3>(12,12) = L_44_gain*_I3x3;

    _K_d.block<3,3>(0,0) = K_00_gain*_I3x3;
    _K_d.block<3,3>(3,3) = K_11_gain*_I3x3;
    _K_d.block<3,3>(6,6) = K_22_gain*_I3x3;
    _K_d.block<3,3>(9,9) = K_33_gain*_I3x3; 

    for(int i=0; i<(MPC_Step+1); i++)
        _L_qp.block<15,15>(15*i,15*i) = _L_d;

    for(int i=0; i<MPC_Step; i++)
        _K_qp.block<12,12>(12*i,12*i) = _K_d;        

    _H_qp = _B_qp.transpose()*_L_qp*_B_qp + _K_qp;
    
    _x0(0) = 0.0;
    _x0(1) = 0.0;
    _x0(2) = EulerAngle(2);
    _x0(3) = _p_com(0);
    _x0(4) = _p_com(1);
    _x0(5) = _p_com(2);
    _x0(6) = _w_body(0);
    _x0(7) = _w_body(1);
    _x0(8) = _w_body(2);
    _x0(9) = _p_com_dot(0);
    _x0(10) = _p_com_dot(1);
    _x0(11) = _p_com_dot(2);
    _x0(12) = 0.0;
    _x0(13) = 0.0;
    _x0(14) = -Gravity;

    _xref(0) = 0.0;
    _xref(1) = 0.0;
    _xref(2) = 0.0;
    _xref(3) = 0.0;
    _xref(4) = 0.0;
    _xref(5) = 0.0;
    _xref(6) = 0.0;
    _xref(7) = 0.0;
    _xref(8) = 0.0;
    _xref(9) = 0.0;
    _xref(10) =0.0;
    _xref(11) = 0.0;
    _xref(12) = 0.0;
    _xref(13) = 0.0;
    _xref(14) = -Gravity;

    _g_qp = 2*_B_qp.transpose()*_L_qp*_A_qp*(_x0 - _xref);

    // Inequality constraint

    _C_1leg <<  0,  0,   -1,
                0,  0,    1,
               -1,  0, -_mu,
                1,  0, -_mu,
                0, -1, -_mu,
                0,  1, -_mu;

    for(int i=0; i<4; i++)
        _C_4leg.block<6,3>(6*i,0) = _C_1leg;  

    for(int i=0; i<MPC_Step; i++)
        _C_qp.block<6*4,3>(6*4*i,3*i) = _C_4leg;

    _lbC_1leg << -Force_min,
                  Force_max,
                          0,
                          0,
                          0,
                          0;

    for(int i=0; i<4; i++)
        _lbC_4leg.block<6,1>(6*i,1*0) = _lbC_1leg;     

    for(int i=0; i<MPC_Step; i++)
        _lbC_qp.block<6*4,1>(6*4*i,1*i) = _lbC_4leg;                    

    // Optimization(QP Solver)

    USING_NAMESPACE_QPOASES

    real_t H_qp_qpoases[(12*MPC_Step)*(12*MPC_Step)] = {0, };
    real_t g_qp_qpoases[(12*MPC_Step)*(1)] = {0, };
    real_t C_qp_qpoases[(6*4*MPC_Step)*(3*MPC_Step)] = {0, };
    real_t lbC_qp_qpoases[(6*4*MPC_Step)*(1*MPC_Step)] = {0, };

    for(int i=0; i<(12*MPC_Step)*(12*MPC_Step); i++)
        H_qp_qpoases[i] = _H_qp(i);

    for(int i=0; i<(12*MPC_Step)*(1); i++)
        g_qp_qpoases[i] = _g_qp(i);

    for(int i=0; i<(6*4*MPC_Step)*(3*MPC_Step); i++)
        C_qp_qpoases[i] = _C_qp(i);

    for(int i=0; i<(6*4*MPC_Step)*(1*MPC_Step); i++)
        lbC_qp_qpoases[i] = _lbC_qp(i);             

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