#include <legged_controllers/mpc_controller.h>

void MPCController::init(double m_body, const KDL::RotationalInertia& I_com, double mu)
{
    _m_body = m_body;
    _I_com = I_com;
    _mu = mu;
}

void MPCController::setControlInput(const KDL::Vector& p_com_d, 
            const KDL::Vector& p_com_dot_d, 
            const KDL::Rotation& R_body_d, 
            const KDL::Vector& w_body_d,
			const KDL::Vector& p_com,
            const KDL::Vector& p_com_dot, 
            const std::array<KDL::Vector,4>& p_leg,
            const KDL::Rotation& R_body,
            const KDL::Vector w_body)
{
    _p_com_d = p_com_d;
    _p_com_dot_d = p_com_dot_d;
    _R_body_d = R_body_d;
    _w_body_d = w_body_d;
    _p_com = p_com;
    _p_com = p_com_dot;
    _p_leg = p_leg;
    _R_body = R_body;
    _w_body = w_body;
}

void MPCController::getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg)
{
    F_leg;
}

void MPCController::update()
{
    // Not tested yet
  
    // Simplified Robot Dynamics x_dot = Ax + Bu
    Eigen::MatrixXf A_c = Eigen::MatrixXf::Zero(15,15);
    Eigen::MatrixXf A_d = Eigen::MatrixXf::Zero(15,15);
    Eigen::MatrixXf B_c = Eigen::MatrixXf::Zero(15,15);
    Eigen::MatrixXf B_d = Eigen::MatrixXf::Zero(15,15);

    Eigen::MatrixXf I3x3 = Eigen::MatrixXf::Identity(3,3);
    Eigen::MatrixXf I15x15 = Eigen::MatrixXf::Identity(15,15);

    Eigen::MatrixXf Rz_m = Eigen::MatrixXf::Identity(3,3);

    Eigen::MatrixXf I_hat = Eigen::MatrixXf::Zero(3,3);
    
    Eigen::MatrixXf p_leg_1_skew = Eigen::MatrixXf::Zero(3,3);
    Eigen::MatrixXf p_leg_2_skew = Eigen::MatrixXf::Zero(3,3);
    Eigen::MatrixXf p_leg_3_skew = Eigen::MatrixXf::Zero(3,3);
    Eigen::MatrixXf p_leg_4_skew = Eigen::MatrixXf::Zero(3,3);

    Eigen::MatrixXf T15X15 = Eigen::MatrixXf::Identity(15,15);

    KDL::Rotation Rz;
    Rz = Rz.Identity();
    double EulerAngle_Z=0.0;
    double EulerAngle_Y=0.0;
    double EulerAngle_X=0.0;

    _R_body_d.GetEulerZYX(EulerAngle_Z,EulerAngle_Y,EulerAngle_X);
    Rz = Rz.EulerZYZ(EulerAngle_Z,0.0,0.0);
    Rz_m << Rz.data[0],Rz.data[1],Rz.data[2],
            Rz.data[3],Rz.data[4],Rz.data[5],
            Rz.data[6],Rz.data[7],Rz.data[8];

    A_c.block<3,3>(0,6) = Rz_m;
    A_c.block<3,3>(3,9) = I3x3;
    A_c.block<3,3>(9,12) = -I3x3;

    I_hat << _I_com.data[0],_I_com.data[1],_I_com.data[2],
             _I_com.data[3],_I_com.data[4],_I_com.data[5],
             _I_com.data[6],_I_com.data[7],_I_com.data[8];

    I_hat = Rz_m*I_hat*Rz_m.transpose();

    skew_symmetric(_p_leg[0],p_leg_1_skew);
    skew_symmetric(_p_leg[1],p_leg_2_skew);
    skew_symmetric(_p_leg[2],p_leg_3_skew);
    skew_symmetric(_p_leg[3],p_leg_4_skew);

    B_c.block<3,3>(6,0) = I_hat.inverse()*p_leg_1_skew;
    B_c.block<3,3>(6,3) = I_hat.inverse()*p_leg_2_skew;
    B_c.block<3,3>(6,6) = I_hat.inverse()*p_leg_3_skew;
    B_c.block<3,3>(6,9) = I_hat.inverse()*p_leg_4_skew;

    B_c.block<3,3>(9,0) = (1/_m_body)*I3x3;
    B_c.block<3,3>(9,3) = (1/_m_body)*I3x3;
    B_c.block<3,3>(9,6) = (1/_m_body)*I3x3;
    B_c.block<3,3>(9,9) = (1/_m_body)*I3x3;

    A_d = I15x15+A_c*(SamplingTime*T15X15);
    B_d = B_c*(SamplingTime*T15X15);

    // Condensed QP formulation y = Aqp*x0 + Bqp*u

    Eigen::MatrixXf A_qp = Eigen::MatrixXf::Zero(15*(MPC_Step+1), 15);
    A_qp.block<15,15>(0,0) = I15x15;

    Eigen::MatrixXf Temp = Eigen::MatrixXf::Zero(15,15);
    Temp = A_d;

    for(int i=1; i<=MPC_Step; i++)
    {
        A_qp.block<15,15>(15*i,0) = Temp;
        Temp *= A_d; 
    }

    Temp = B_d;
    Eigen::MatrixXf B_qp = Eigen::MatrixXf::Zero(15*(MPC_Step+1), 15*MPC_Step);

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

    Eigen::MatrixXf L_d = Eigen::MatrixXf::Identity(15,15);
    Eigen::MatrixXf K_d = Eigen::MatrixXf::Identity(15,15);

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

    Eigen::MatrixXf H_qp = Eigen::MatrixXf::Zero(15*MPC_Step, 15*MPC_Step);
    Eigen::MatrixXf L_qp = Eigen::MatrixXf::Zero(15*(MPC_Step+1), 15*(MPC_Step+1));
    Eigen::MatrixXf K_qp = Eigen::MatrixXf::Zero(15*MPC_Step, 15*MPC_Step);

    for(int i=0; i<(MPC_Step+1); i++)
        L_qp.block<15,15>(15*i,15*i) = L_d;

    for(int i=0; i<MPC_Step; i++)
        K_qp.block<15,15>(15*i,15*i) = K_d;        

    H_qp = B_qp.transpose()*L_qp*B_qp + K_qp;
    
    Eigen::MatrixXf g_qp = Eigen::MatrixXf::Zero(15*MPC_Step, 1);
    Eigen::MatrixXf x0 = Eigen::MatrixXf::Zero(15, 1);

    x0(0,0) = 0.0; //EulerAngle_X;
    x0(0,1) = 0.0; //EulerAngle_Y;
    x0(0,2) = EulerAngle_Z;
    x0(0,3) = _p_com(0);
    x0(0,4) = _p_com(1);
    x0(0,5) = _p_com(2);
    x0(0,6) = _w_body(0);
    x0(0,7) = _w_body(1);
    x0(0,8) = _w_body(2);
    x0(0,9) = _p_com_dot(0);
    x0(0,10) = _p_com_dot(1);
    x0(0,11) = _p_com_dot(2);
    x0(0,12) = 0.0;
    x0(0,13) = 0.0;
    x0(0,14) = -9.8;

    g_qp = B_qp.transpose()*L_qp*A_qp*x0;

    // Optimization
    USING_NAMESPACE_QPOASES

   real_t H_qp_opoases[(15*MPC_Step)*(15*MPC_Step)];
   real_t g_qp_opoases[(15*MPC_Step)*(1)];

 /* 
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
 */     
}

void MPCController::skew_symmetric(KDL::Vector &v_,Eigen::MatrixXf skew_mat_)
{
	skew_mat_ = Eigen::MatrixXf::Zero(3,3);
	
	skew_mat_(0,1) = -v_(2);
	skew_mat_(0,2) =  v_(1);
	skew_mat_(1,0) =  v_(2);
	skew_mat_(1,2) = -v_(0);
	skew_mat_(2,0) = -v_(1);
	skew_mat_(2,1) =  v_(0);
}