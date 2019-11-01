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
    _B_c_d = Eigen::MatrixXd::Zero(15,12);

    _B_d = Eigen::MatrixXd::Zero(15,12);
    _B_d_d = Eigen::MatrixXd::Zero(15,12);

    _I3x3 = Eigen::MatrixXd::Identity(3,3);
    _I15x15 = Eigen::MatrixXd::Identity(15,15);

    _I_hat = Eigen::MatrixXd::Zero(3,3);
    
    _p_leg_1_skew = Eigen::MatrixXd::Zero(3,3);
    _p_leg_2_skew = Eigen::MatrixXd::Zero(3,3);
    _p_leg_3_skew = Eigen::MatrixXd::Zero(3,3);
    _p_leg_4_skew = Eigen::MatrixXd::Zero(3,3);

    _p_leg_1_skew_d = Eigen::MatrixXd::Zero(3,3);
    _p_leg_2_skew_d = Eigen::MatrixXd::Zero(3,3);
    _p_leg_3_skew_d = Eigen::MatrixXd::Zero(3,3);
    _p_leg_4_skew_d = Eigen::MatrixXd::Zero(3,3);

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
    _xref_qp = Eigen::MatrixXd::Zero(15*(MPC_Step+1), 1);  

    _C_1leg = Eigen::MatrixXd::Zero(4,3);
    _C_4leg = Eigen::MatrixXd::Zero(4*4,3);
    _C_qp = Eigen::MatrixXd::Zero(4*4*MPC_Step,3*MPC_Step); 

    _lbC_1leg = Eigen::MatrixXd::Zero(4,1);
    _lbC_4leg = Eigen::MatrixXd::Zero(4*4,1);
    _lbC_qp = Eigen::MatrixXd::Zero(4*4*MPC_Step,1);

    _ub_1leg = Eigen::MatrixXd::Zero(3,1);
    _ub_4leg = Eigen::MatrixXd::Zero(3*4,1);
    _ub_qp = Eigen::MatrixXd::Zero(3*4*MPC_Step,1);

    _lb_1leg = Eigen::MatrixXd::Zero(3,1);
    _lb_4leg = Eigen::MatrixXd::Zero(3*4,1);
    _lb_qp = Eigen::MatrixXd::Zero(3*4*MPC_Step,1);               
}

void MPCController::setControlInput(const Eigen::Vector3d& p_body_d, 
            const Eigen::Vector3d& p_body_dot_d, 
            const Eigen::Matrix3d& R_body_d, 
            const Eigen::Vector3d& w_body_d,
            const std::array<Eigen::Vector3d,4>& p_body2leg_d,
			const Eigen::Vector3d& p_body, 
            const Eigen::Vector3d& p_body_dot,
            const Eigen::Matrix3d& R_body,
            const Eigen::Vector3d w_body,
            const std::array<Eigen::Vector3d,4>& p_body2leg)
{
    // to world coordinates
    for (int i=0; i<4; i++)
        _p_leg[i] = p_body + R_body*p_body2leg[i];

    for (int i=0; i<4; i++)
        _p_leg_d[i] = p_body_d + R_body_d*p_body2leg_d[i];    

    // rotation
    _w_body_d = w_body_d;
    _w_body = w_body;

    _R_body_d = R_body_d;
    _R_body = R_body;

    // body to com
    _p_com_d = p_body_d + R_body_d * _p_body2com;
    _p_com = p_body + R_body * _p_body2com;
    _p_com_dot_d = p_body_dot_d + skew(w_body_d) * R_body_d * _p_body2com;
    _p_com_dot = p_body_dot + skew(w_body) * R_body * _p_body2com;
}

void MPCController::getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg)
{
    F_leg[0] = -_R_body.transpose()*_F.segment(0,3);
    F_leg[1] = -_R_body.transpose()*_F.segment(3,3);
    F_leg[2] = -_R_body.transpose()*_F.segment(6,3);
    F_leg[3] = -_R_body.transpose()*_F.segment(9,3);
}

void MPCController::update()
{
    // Not tested yet

    // Continuous Simplified Robot Dynamics x_dot = A_c*x + B_c*u

    Eigen::Vector3d EulerAngle = _R_body.eulerAngles(2, 1, 0); 
    Eigen::Matrix3d Rz;
    Rz = AngleAxisd(EulerAngle(2), Vector3d::UnitZ());

    Eigen::Vector3d EulerAngle_d = _R_body_d.eulerAngles(2, 1, 0); 
    
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

    _p_leg_1_skew_d = skew(_p_leg_d[0]);
    _p_leg_2_skew_d = skew(_p_leg_d[1]);
    _p_leg_3_skew_d = skew(_p_leg_d[2]);
    _p_leg_4_skew_d = skew(_p_leg_d[3]);

    _B_c_d.block<3,3>(6,0) = _I_hat.inverse()*_p_leg_1_skew_d;
    _B_c_d.block<3,3>(6,3) = _I_hat.inverse()*_p_leg_2_skew_d;
    _B_c_d.block<3,3>(6,6) = _I_hat.inverse()*_p_leg_3_skew_d;
    _B_c_d.block<3,3>(6,9) = _I_hat.inverse()*_p_leg_4_skew_d;

    _B_c_d.block<3,3>(9,0) = (1/_m_body)*_I3x3;
    _B_c_d.block<3,3>(9,3) = (1/_m_body)*_I3x3;
    _B_c_d.block<3,3>(9,6) = (1/_m_body)*_I3x3;
    _B_c_d.block<3,3>(9,9) = (1/_m_body)*_I3x3;    

    // Discrete Simplified Robot Dynamics x[k+1] = A_c*x[k] + B_c*u[k]

    _A_d = _I15x15+_A_c*(SamplingTime*_T15X15);
    _B_d = _B_c*(SamplingTime*_T12X12);
    _B_d_d = _B_c_d*(SamplingTime*_T12X12);

    // Condensed QP formulation X = Aqp*x0 + Bqp*U

    _A_qp.block<15,15>(0,0) = _I15x15;

    _Temp = _A_d;

    for(int i=1; i<=MPC_Step; i++)
    {
        _A_qp.block<15,15>(15*i,0) = _Temp;
        _Temp *= _A_d; 
    }

    for(int i=0; i<MPC_Step; i++)
    {
        _Temp = _B_d;   

        for(int j=(i+1); j<=MPC_Step; j++)
        {
            _B_qp.block<15,12>(15*j,12*i) = _Temp;

            _Temp = _A_d*_Temp;
        }   
    }

    //_B_qp.block<15,12>(15*1,12*0) = _B_d_d;

    _L_d.block<3,3>(0,0) = L_00_gain*_I3x3;

    Eigen::Matrix3d L_11_gain_xyz;
    L_11_gain_xyz = _I3x3;
    L_11_gain_xyz(0,0) = L_11_gain_x*_I3x3(0,0);
    L_11_gain_xyz(1,1) = L_11_gain_y*_I3x3(1,1);
    L_11_gain_xyz(2,2) = L_11_gain_z*_I3x3(2,2);

    Eigen::Matrix3d L_22_gain_wxyz;
    L_22_gain_wxyz = _I3x3;
    L_22_gain_wxyz(0,0) = L_22_gain_wx*_I3x3(0,0);
    L_22_gain_wxyz(1,1) = L_22_gain_wy*_I3x3(1,1);
    L_22_gain_wxyz(2,2) = L_22_gain_wz*_I3x3(2,2);

    Eigen::Matrix3d L_33_gain_vxyz;
    L_33_gain_vxyz = _I3x3;
    L_33_gain_vxyz(0,0) = L_33_gain_vx*_I3x3(0,0);
    L_33_gain_vxyz(1,1) = L_33_gain_vy*_I3x3(1,1);
    L_33_gain_vxyz(2,2) = L_33_gain_vz*_I3x3(2,2);

    _L_d.block<3,3>(3,3) = L_11_gain_xyz;
    _L_d.block<3,3>(6,6) = L_22_gain_wxyz*_I3x3;
    _L_d.block<3,3>(9,9) = L_33_gain_vxyz*_I3x3;
    _L_d.block<3,3>(12,12) = L_44_gain*_I3x3;

    _K_d.block<3,3>(0,0) = K_00_gain*_I3x3;
    _K_d.block<3,3>(3,3) = K_11_gain*_I3x3;
    _K_d.block<3,3>(6,6) = K_22_gain*_I3x3;
    _K_d.block<3,3>(9,9) = K_33_gain*_I3x3; 

    for(int i=0; i<(MPC_Step+1); i++)
        _L_qp.block<15,15>(15*i,15*i) = _L_d;

    for(int i=0; i<MPC_Step; i++)
        _K_qp.block<12,12>(12*i,12*i) = _K_d;        

    _H_qp = 2*(_B_qp.transpose()*_L_qp*_B_qp + _K_qp);
    
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
    _x0(14) = Gravity;

    _xref(0) = 0.0;
    _xref(1) = 0.0;
    _xref(2) = EulerAngle_d(2);
    _xref(3) = _p_com_d(0);
    _xref(4) = _p_com_d(1);
    _xref(5) = _p_com_d(2);
    _xref(6) = _w_body_d(0);
    _xref(7) = _w_body_d(1);
    _xref(8) = _w_body_d(2);
    _xref(9) = _p_com_dot_d(0);
    _xref(10) = _p_com_dot_d(1);
    _xref(11) = _p_com_dot_d(2);
    _xref(12) = 0.0;
    _xref(13) = 0.0;
    _xref(14) = Gravity;

    for(int i=0; i<(MPC_Step+1); i++)
    {
        _xref_qp.block<15,1>(15*i,0) = _xref;
    }

    _g_qp = 2*_B_qp.transpose()*_L_qp*(_A_qp*_x0 - _xref_qp);

    // Inequality constraint

    Vector3d e1(_mu/sqrt(2*_mu*_mu+1),_mu/sqrt(2*_mu*_mu+1),1/sqrt(2*_mu*_mu+1));
    Vector3d e2(-_mu/sqrt(2*_mu*_mu+1),_mu/sqrt(2*_mu*_mu+1),1/sqrt(2*_mu*_mu+1));
    Vector3d e3(-_mu/sqrt(2*_mu*_mu+1),-_mu/sqrt(2*_mu*_mu+1),1/sqrt(2*_mu*_mu+1));
    Vector3d e4(_mu/sqrt(2*_mu*_mu+1),-_mu/sqrt(2*_mu*_mu+1),1/sqrt(2*_mu*_mu+1));
    
    Vector3d e1Xe2 = e1.cross(e2);
    Vector3d e2Xe3 = e2.cross(e3);
    Vector3d e3Xe4 = e3.cross(e4);
    Vector3d e4Xe1 = e4.cross(e1);

    // method 1 
    //_C_1leg << e1(0), e1(1), e1(2),
    //           e2(0), e2(1), e2(2),
    //           e3(0), e3(1), e3(2),
    //           e4(0), e4(1), e4(2);  

    // method 2
    _C_1leg << e1Xe2(0), e1Xe2(1), e1Xe2(2),
               e2Xe3(0), e2Xe3(1), e2Xe3(2),
               e3Xe4(0), e3Xe4(1), e3Xe4(2),
               e4Xe1(0), e4Xe1(1), e4Xe1(2);           

    for(int i=0; i<4; i++)
        _C_4leg.block<4,3>(4*i,0) = _C_1leg;  

    for(int i=0; i<MPC_Step; i++)
        _C_qp.block<4*4,3>(4*4*i,3*i) = _C_4leg;

    _lbC_1leg << 0,
                 0,
                 0,
                 0;

    for(int i=0; i<4; i++)
        _lbC_4leg.block<4,1>(4*i,0) = _lbC_1leg;     

    for(int i=0; i<MPC_Step; i++)
        _lbC_qp.block<4*4,1>(4*4*i,0) = _lbC_4leg; 

    _ub_1leg << _mu*Force_max, _mu*Force_max, Force_max;

    for(int i=0; i<4; i++)
        _ub_4leg.block<3,1>(3*i,0) = _ub_1leg;     

    for(int i=0; i<MPC_Step; i++)
        _ub_qp.block<3*4,1>(3*4*i,0) = _ub_4leg;   

    _lb_1leg << -_mu*Force_max, -_mu*Force_max, Force_min;

    for(int i=0; i<4; i++)
        _lb_4leg.block<3,1>(3*i,0) = _lb_1leg;     

    for(int i=0; i<MPC_Step; i++)
        _lb_qp.block<3*4,1>(3*4*i,0) = _lb_4leg;    

    // Optimization(QP Solver)

    USING_NAMESPACE_QPOASES
    real_t H_qp_qpoases[(12*MPC_Step)*(12*MPC_Step)] = {0, };
    real_t g_qp_qpoases[(12*MPC_Step)*(1)] = {0, };

    real_t C_qp_qpoases[(4*4*MPC_Step)*(3*MPC_Step)] = {0, };
    real_t lbC_qp_qpoases[4*4*MPC_Step] = {0, };
    real_t ub_qp_qpoases[3*4*MPC_Step] = {0, };
    real_t lb_qp_qpoases[3*4*MPC_Step] = {0, };    

    for(int i=0; i<(12*MPC_Step); i++)
        for(int j=0; j<(12*MPC_Step); j++)
            H_qp_qpoases[i*(12*MPC_Step)+j] = _H_qp(i,j);
    
    for(int i=0; i<(12*MPC_Step); i++)
        for(int j=0; j<(1); j++)
            g_qp_qpoases[i*(1)+j] = _g_qp(i,j);       

    for(int i=0; i<(4*4*MPC_Step); i++)
        for(int j=0; j<(3*MPC_Step); j++)
            C_qp_qpoases[i*(3*MPC_Step)+j] = _C_qp(i,j);

    for(int i=0; i<(4*4*MPC_Step); i++)
        lbC_qp_qpoases[i] = _lbC_qp(i,0); 

    for(int i=0; i<(3*4*MPC_Step); i++)
        ub_qp_qpoases[i] = _ub_qp(i,0); 

    for(int i=0; i<(3*4*MPC_Step); i++)
        lb_qp_qpoases[i] = _lb_qp(i,0);               

    QProblem qp_problem( 12*MPC_Step, 1 );

    Options options;
	qp_problem.setOptions( options );

    int nWSR = 1000;
 
    qp_problem.init(H_qp_qpoases, g_qp_qpoases, C_qp_qpoases, lb_qp_qpoases, ub_qp_qpoases, lbC_qp_qpoases, NULL, nWSR);

	real_t UOpt[12*MPC_Step];
    qp_problem.getPrimalSolution( UOpt );

    _F << UOpt[0], UOpt[1],  UOpt[2],
          UOpt[3], UOpt[4],  UOpt[5],
          UOpt[6], UOpt[7],  UOpt[8],
          UOpt[9], UOpt[10], UOpt[11];         
}
