#include <legged_controllers/mpc_controller.h>

void MPCController::init()
{
}

void MPCController::setControlData(quadruped_robot::QuadrupedRobot &robot)
{
    // RobotData Update
    _m_body = robot._m_body;
    _mu = robot._mu_foot;
    _I_com = robot._I_com_body;
    _p_com_d = robot._pose_com_d._pos;
    _p_com = robot._pose_com._pos;
    _p_com_dot_d = robot._pose_vel_com_d._linear;
    _p_com_dot = robot._pose_vel_com._linear;
    _w_body_d = robot._pose_vel_body_d._angular;
    _w_body = robot._pose_vel_body._angular;
    _R_body_d = robot._pose_body_d._rot_quat.toRotationMatrix();
    _R_body = robot._pose_body._rot_quat.toRotationMatrix();
    _p_leg_d = robot._p_world2leg_d;
    _p_leg = robot._p_world2leg;
    const std::array<int, 4>& contact_states = robot._contact_states;

    // LegContactState Update
    _LegContactState.ContactTotalNum = 0;

    for(int i=0; i<4; i++)
        _LegContactState.LegState[i] = false;

    for (int i = 0; i < 4; i++)
    {
        if (robot.getController(i) == quadruped_robot::controllers::BalancingMPC && contact_states[i] == 1)
        {
            _LegContactState.LegState[i] = true;
            _LegContactState.ContactTotalNum++;
        }
    }

    if (_LegContactState.ContactTotalNum < 1)
        return;

    // Matrix Resize according to LegContactState
    _R1 = Eigen::MatrixXd::Zero(3, 3);
    _R2 = Eigen::MatrixXd::Zero(3, 3);
    _R3 = Eigen::MatrixXd::Zero(3, 3);
    _R4 = Eigen::MatrixXd::Zero(3, 3);

    _A_d = Eigen::MatrixXd::Zero(15, 15);    
    _B_d = Eigen::MatrixXd::Zero(15, 3*_LegContactState.ContactTotalNum);
    _B_d_d = Eigen::MatrixXd::Zero(15, 3*_LegContactState.ContactTotalNum);

    _Rz = Eigen::MatrixXd::Zero(3, 3);
    _Rz_d = Eigen::MatrixXd::Zero(3, 3);

    _I3x3 = Eigen::MatrixXd::Identity(3, 3);
    _I15x15 = Eigen::MatrixXd::Identity(15, 15);

    _I_hat = Eigen::MatrixXd::Zero(3, 3);
    _I_hat_d = Eigen::MatrixXd::Zero(3, 3);

    _p_leg_1_skew = Eigen::MatrixXd::Zero(3, 3);
    _p_leg_2_skew = Eigen::MatrixXd::Zero(3, 3);
    _p_leg_3_skew = Eigen::MatrixXd::Zero(3, 3);
    _p_leg_4_skew = Eigen::MatrixXd::Zero(3, 3);

    _p_leg_1_skew_d = Eigen::MatrixXd::Zero(3, 3);
    _p_leg_2_skew_d = Eigen::MatrixXd::Zero(3, 3);
    _p_leg_3_skew_d = Eigen::MatrixXd::Zero(3, 3);
    _p_leg_4_skew_d = Eigen::MatrixXd::Zero(3, 3); 

    _A_qp = Eigen::MatrixXd::Zero(15 * (MPC_Step + 1), 15);
    _Temp = Eigen::MatrixXd::Zero(15, 15);

    _B_qp = Eigen::MatrixXd::Zero(15 * (MPC_Step + 1), 3*_LegContactState.ContactTotalNum*MPC_Step);

    _L_d = Eigen::MatrixXd::Identity(15, 15);
    _K_d = Eigen::MatrixXd::Identity(3*_LegContactState.ContactTotalNum, 3*_LegContactState.ContactTotalNum);

    _H_qp = Eigen::MatrixXd::Zero(3*_LegContactState.ContactTotalNum * MPC_Step, 3*_LegContactState.ContactTotalNum * MPC_Step);
    _L_qp = Eigen::MatrixXd::Zero(15 * (MPC_Step + 1), 15 * (MPC_Step + 1));
    _K_qp = Eigen::MatrixXd::Zero(3*_LegContactState.ContactTotalNum * MPC_Step, 3*_LegContactState.ContactTotalNum * MPC_Step);   

    _g_qp = Eigen::MatrixXd::Zero(3*_LegContactState.ContactTotalNum*MPC_Step, 1);
    _x0 = Eigen::MatrixXd::Zero(15, 1);
    _xref = Eigen::MatrixXd::Zero(15, 1);
    _xref_qp = Eigen::MatrixXd::Zero(15 * (MPC_Step + 1), 1);    

    _C_1leg = Eigen::MatrixXd::Zero(4, 3);
    _C_totalleg = Eigen::MatrixXd::Zero(4 * _LegContactState.ContactTotalNum, 3);
    _C_qp = Eigen::MatrixXd::Zero(4 * _LegContactState.ContactTotalNum * MPC_Step, 3 * MPC_Step);

    _lbC_1leg = Eigen::MatrixXd::Zero(4, 1);
    _lbC_totalleg = Eigen::MatrixXd::Zero(4 * _LegContactState.ContactTotalNum, 1);
    _lbC_qp = Eigen::MatrixXd::Zero(4 * _LegContactState.ContactTotalNum * MPC_Step, 1);

    _ub_1leg = Eigen::MatrixXd::Zero(3, 1);
    _ub_totalleg = Eigen::MatrixXd::Zero(3 * _LegContactState.ContactTotalNum, 1);
    _ub_qp = Eigen::MatrixXd::Zero(3 * _LegContactState.ContactTotalNum * MPC_Step, 1);

    _lb_1leg = Eigen::MatrixXd::Zero(3, 1);
    _lb_totalleg = Eigen::MatrixXd::Zero(3 * _LegContactState.ContactTotalNum, 1);
    _lb_qp = Eigen::MatrixXd::Zero(3 * _LegContactState.ContactTotalNum * MPC_Step, 1);
}

void MPCController::calControlInput()
{
    if (_LegContactState.ContactTotalNum < 1)
        return;
            
    // Continuous Simplified Robot Dynamics x_dot = A_c*x + B_c*u

    Eigen::Matrix3d Rz, Rz_d;

    Eigen::Vector3d EulerAngle = _R_body.eulerAngles(2, 1, 0);
    Rz = AngleAxisd(EulerAngle(2), Vector3d::UnitZ());

    Eigen::Vector3d EulerAngle_d = _R_body_d.eulerAngles(2, 1, 0);
    Rz_d = AngleAxisd(EulerAngle_d(2), Vector3d::UnitZ());

    _Rz = Rz;
    _Rz_d = Rz_d;

    _I_hat = _I_com;
    _I_hat = _Rz * _I_hat * _Rz.transpose();
    _I_hat_d = _Rz_d * _I_hat * _Rz_d.transpose();

    _p_leg_1_skew = skew(_p_leg[0] - _p_com);
    _p_leg_2_skew = skew(_p_leg[1] - _p_com);
    _p_leg_3_skew = skew(_p_leg[2] - _p_com);
    _p_leg_4_skew = skew(_p_leg[3] - _p_com);

    _R1 = _I_hat.inverse() * _p_leg_1_skew;
    _R2 = _I_hat.inverse() * _p_leg_2_skew;
    _R3 = _I_hat.inverse() * _p_leg_3_skew;
    _R4 = _I_hat.inverse() * _p_leg_4_skew;

    _p_leg_1_skew_d = skew(_p_leg_d[0] - _p_com_d);
    _p_leg_2_skew_d = skew(_p_leg_d[1] - _p_com_d);
    _p_leg_3_skew_d = skew(_p_leg_d[2] - _p_com_d);
    _p_leg_4_skew_d = skew(_p_leg_d[3] - _p_com_d);

    _R1_d = _I_hat_d.inverse() * _p_leg_1_skew_d;
    _R2_d = _I_hat_d.inverse() * _p_leg_2_skew_d;
    _R3_d = _I_hat_d.inverse() * _p_leg_3_skew_d;
    _R4_d = _I_hat_d.inverse() * _p_leg_4_skew_d;

    // Discrete Simplified Robot Dynamics x[k+1] = A_c*x[k] + B_c*u[k]

    cal_A_d();
    cal_B_d_and_B_d_d();

    // Condensed QP formulation X = Aqp*x0 + Bqp*U

    _A_qp.block<15, 15>(0, 0) = _I15x15;

    _Temp = _A_d;

    for (int i = 1; i <= MPC_Step; i++)
    {
        _A_qp.block<15, 15>(15 * i, 0) = _Temp;
        _Temp = _Temp * _A_d;
    }

    for (int i = 0; i < MPC_Step; i++)
    {
        _Temp = _B_d_d;

        for (int j = (i + 1); j <= MPC_Step; j++)
        {
            for(int k=0; k<15; k++)
            {
                for(int l=0; l<3 * _LegContactState.ContactTotalNum; l++)
                {
                    _B_qp(15*j+k,3 * _LegContactState.ContactTotalNum * i+l) = _Temp(k,l);    
                }
            }

            _Temp = _A_d * _Temp;
        }
    }

    for (int k = 0; k < 15; k++)
    {
        for (int l = 0; l < 3 * _LegContactState.ContactTotalNum; l++)
        {
            _B_qp(15 * 1+k, 3 * _LegContactState.ContactTotalNum * 0+l) = _B_d(k,l);
        }
    }

    _L_d.block<3, 3>(0, 0) = L_00_gain * _I3x3;

    Eigen::Matrix3d L_11_gain_xyz;
    L_11_gain_xyz = _I3x3;
    L_11_gain_xyz(0, 0) = L_11_gain_x * _I3x3(0, 0);
    L_11_gain_xyz(1, 1) = L_11_gain_y * _I3x3(1, 1);
    L_11_gain_xyz(2, 2) = L_11_gain_z * _I3x3(2, 2);

    Eigen::Matrix3d L_22_gain_wxyz;
    L_22_gain_wxyz = _I3x3;
    L_22_gain_wxyz(0, 0) = L_22_gain_wx * _I3x3(0, 0);
    L_22_gain_wxyz(1, 1) = L_22_gain_wy * _I3x3(1, 1);
    L_22_gain_wxyz(2, 2) = L_22_gain_wz * _I3x3(2, 2);

    Eigen::Matrix3d L_33_gain_vxyz;
    L_33_gain_vxyz = _I3x3;
    L_33_gain_vxyz(0, 0) = L_33_gain_vx * _I3x3(0, 0);
    L_33_gain_vxyz(1, 1) = L_33_gain_vy * _I3x3(1, 1);
    L_33_gain_vxyz(2, 2) = L_33_gain_vz * _I3x3(2, 2);

    _L_d.block<3, 3>(3, 3) = L_11_gain_xyz;
    _L_d.block<3, 3>(6, 6) = L_22_gain_wxyz * _I3x3;
    _L_d.block<3, 3>(9, 9) = L_33_gain_vxyz * _I3x3;
    _L_d.block<3, 3>(12, 12) = L_44_gain * _I3x3;

    _K_d = K_gain*_K_d;

    for (int i = 0; i < (MPC_Step + 1); i++)
        _L_qp.block<15, 15>(15 * i, 15 * i) = _L_d;

    for (int i = 0; i < MPC_Step; i++)
    {
        for(int k = 0; k < 3*_LegContactState.ContactTotalNum; k++)
        {
            for(int l = 0; l < 3*_LegContactState.ContactTotalNum; l++)
            {
                _K_qp(3*_LegContactState.ContactTotalNum * i+k, 3*_LegContactState.ContactTotalNum * i+l) = _K_d(k,l);
            }
        }

    }

    _H_qp = 2 * (_B_qp.transpose() * _L_qp * _B_qp + _K_qp);

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

    for (int i = 0; i < (MPC_Step + 1); i++)
    {
        _xref_qp.block<15, 1>(15 * i, 0) = _xref;
    }

    _g_qp = 2 * _B_qp.transpose() * _L_qp * (_A_qp * _x0 - _xref_qp);

    // Inequality constraint

    Vector3d e1(_mu / sqrt(2 * _mu * _mu + 1), _mu / sqrt(2 * _mu * _mu + 1), 1 / sqrt(2 * _mu * _mu + 1));
    Vector3d e2(-_mu / sqrt(2 * _mu * _mu + 1), _mu / sqrt(2 * _mu * _mu + 1), 1 / sqrt(2 * _mu * _mu + 1));
    Vector3d e3(-_mu / sqrt(2 * _mu * _mu + 1), -_mu / sqrt(2 * _mu * _mu + 1), 1 / sqrt(2 * _mu * _mu + 1));
    Vector3d e4(_mu / sqrt(2 * _mu * _mu + 1), -_mu / sqrt(2 * _mu * _mu + 1), 1 / sqrt(2 * _mu * _mu + 1));

    Vector3d e1Xe2 = e1.cross(e2);
    Vector3d e2Xe3 = e2.cross(e3);
    Vector3d e3Xe4 = e3.cross(e4);
    Vector3d e4Xe1 = e4.cross(e1);

    _C_1leg << e1(0), e1(1), e1(2),
        e2(0), e2(1), e2(2),
        e3(0), e3(1), e3(2),
        e4(0), e4(1), e4(2);

    for (int i = 0; i < _LegContactState.ContactTotalNum; i++)
    {
        for(int k = 0; k < _LegContactState.ContactTotalNum; k++)
        {
            for(int l = 0; l < 3; l++)
            {
                _C_totalleg(_LegContactState.ContactTotalNum * i+k, 0+l) = _C_1leg(k,l);
            }
        }
    }
        

    for (int i = 0; i < MPC_Step; i++)
    {
        for(int k = 0; k < 4 * _LegContactState.ContactTotalNum; k++)
        {
            for(int l = 0; l < 3; l++)
            {
                _C_qp(4 * _LegContactState.ContactTotalNum * i+k, 3 * i+l) = _C_totalleg(k,l);
            }
        }
    }
        
    _lbC_1leg << 0,
        0,
        0,
        0;

    for (int i = 0; i < _LegContactState.ContactTotalNum; i++)
    {
        for(int k = 0; k < _LegContactState.ContactTotalNum; k++)
        {
            for(int l = 0; l < 1; l++)
            {
                _lbC_totalleg(_LegContactState.ContactTotalNum * i+k, 0+l) = _lbC_1leg(k,l);
            }
        }
    }

    for (int i = 0; i < MPC_Step; i++)
    {
        for(int k = 0; k < 4*_LegContactState.ContactTotalNum; k++)
        {
            for(int l =0; l < 1; l++)
            {
                _lbC_qp(4 * _LegContactState.ContactTotalNum * i+k, 0+l) = _lbC_totalleg(k,l);
            }
        }

    }
        
    _ub_1leg << _mu * Force_max, _mu * Force_max, Force_max;

    for (int i = 0; i < _LegContactState.ContactTotalNum; i++)
    {
        for(int k = 0; k < 3; k++)
        {
            for(int l = 0; l < 1; l++)
            {
                 _ub_totalleg(3 * i+k, 0+l) = _ub_1leg(k,l);
            }
        }
    }

    for (int i = 0; i < MPC_Step; i++)
    {
        for(int k = 0; k < 3 * _LegContactState.ContactTotalNum; k++)
        {
            for(int l = 0; l < 1; l++)
            {
                _ub_qp(3 * _LegContactState.ContactTotalNum * i+k, 0+l) = _ub_totalleg(k,l);
            }
        }
    }
        
    _lb_1leg << -_mu * Force_max, -_mu * Force_max, Force_min;

    for (int i = 0; i < _LegContactState.ContactTotalNum; i++)
    {
        for(int k = 0; k < 3; k++)
        {
            for(int l = 0; l < 1; l++)
            {
                _lb_totalleg(3 * i+k, 0+l) = _lb_1leg(k,l);
            }
        }
    }
        

    for (int i = 0; i < MPC_Step; i++)
    {
        for(int k = 0; k < 3 * _LegContactState.ContactTotalNum; k++)
        {
            for(int l = 0; l < 1; l++)
            {
                _lb_qp(3 * _LegContactState.ContactTotalNum * i+k, 0+l) = _lb_totalleg(k,l);
            }
        }
    }

    // Optimization(QP Solver)

    USING_NAMESPACE_QPOASES
    real_t H_qp_qpoases[(3 * _LegContactState.ContactTotalNum * MPC_Step) * (3 * _LegContactState.ContactTotalNum * MPC_Step)] = {
        0,
    };
    real_t g_qp_qpoases[(3 * _LegContactState.ContactTotalNum * MPC_Step) * (1)] = {
        0,
    };

    real_t C_qp_qpoases[(4 * 3 * _LegContactState.ContactTotalNum * MPC_Step) * (3 * MPC_Step)] = {
        0,
    };
    real_t lbC_qp_qpoases[4 * 3 * _LegContactState.ContactTotalNum * MPC_Step] = {
        0,
    };
    real_t ub_qp_qpoases[3 * 3 * _LegContactState.ContactTotalNum * MPC_Step] = {
        0,
    };
    real_t lb_qp_qpoases[3 * 3 * _LegContactState.ContactTotalNum * MPC_Step] = {
        0,
    };

    for (int i = 0; i < (3 * _LegContactState.ContactTotalNum * MPC_Step); i++)
        for (int j = 0; j < (3 * _LegContactState.ContactTotalNum * MPC_Step); j++)
            H_qp_qpoases[i * (3 * _LegContactState.ContactTotalNum * MPC_Step) + j] = _H_qp(i, j);

    for (int i = 0; i < (3 * _LegContactState.ContactTotalNum * MPC_Step); i++)
        for (int j = 0; j < (1); j++)
            g_qp_qpoases[i * (1) + j] = _g_qp(i, j);

    for (int i = 0; i < (4 * _LegContactState.ContactTotalNum * MPC_Step); i++)
        for (int j = 0; j < (3 * MPC_Step); j++)
            C_qp_qpoases[i * (3 * MPC_Step) + j] = _C_qp(i, j);

    for (int i = 0; i < (4 * _LegContactState.ContactTotalNum * MPC_Step); i++)
        lbC_qp_qpoases[i] = _lbC_qp(i, 0);

    for (int i = 0; i < (3 * _LegContactState.ContactTotalNum * MPC_Step); i++)
        ub_qp_qpoases[i] = _ub_qp(i, 0);

    for (int i = 0; i < (3 * _LegContactState.ContactTotalNum * MPC_Step); i++)
        lb_qp_qpoases[i] = _lb_qp(i, 0);

    QProblem qp_problem(3 * _LegContactState.ContactTotalNum * MPC_Step, _LegContactState.ContactTotalNum * MPC_Step);

    Options options;
    qp_problem.setOptions(options);

    int nWSR = 100;

    qp_problem.init(H_qp_qpoases, g_qp_qpoases, C_qp_qpoases, lb_qp_qpoases, ub_qp_qpoases, lbC_qp_qpoases, NULL, nWSR);

    real_t UOpt[3 * _LegContactState.ContactTotalNum * MPC_Step];
    qp_problem.getPrimalSolution(UOpt);

    int select_count = 0;
    int num = 0;

    if (_LegContactState.LegState[0])
    {
        num = select_count * 3;

        _F[0](0) = UOpt[num];
        _F[0](1) = UOpt[num+1];
        _F[0](2) = UOpt[num+2];

        select_count++;
    }

    printf("1 num(%d) ",num);

    if (_LegContactState.LegState[1])
    {
        num = select_count * 3;

        _F[1](0) = UOpt[num];
        _F[1](1) = UOpt[num+1];
        _F[1](2) = UOpt[num+2];

        select_count++;
    }

    printf("2 num(%d) ",num);

    if (_LegContactState.LegState[2])
    {
        num = select_count * 3;

        _F[2](0) = UOpt[num];
        _F[2](1) = UOpt[num+1];
        _F[2](2) = UOpt[num+2];

        select_count++;
    }

    printf("3 num(%d) ",num);

    if (_LegContactState.LegState[3])
    {
        num = select_count * 3;

        _F[3](0) = UOpt[num];
        _F[3](1) = UOpt[num+1];
        _F[3](2) = UOpt[num+2];

        select_count++;
    }     

    printf("4 num(%d) \n",num);        
}

void MPCController::getControlInput(quadruped_robot::QuadrupedRobot &robot, std::array<Eigen::Vector3d, 4> &F_leg)
{
    for (size_t i = 0; i < 4; i++)
    {
        if (robot.getController(i) == quadruped_robot::controllers::BalancingMPC)
        {
            F_leg[i] = -_R_body.transpose() * _F[i];
        }
    }
}

void MPCController::cal_A_d()
{
    _A_d = _I15x15;
    _A_d.block<3, 3>(0, 6) = _Rz * SamplingTime;
    _A_d.block<3, 3>(3, 9) = _I3x3 * SamplingTime;
    _A_d.block<3, 3>(3, 12) = _I3x3 * (-1 / 2 * SamplingTime * SamplingTime);
    _A_d.block<3, 3>(9, 12) = _I3x3 * (-SamplingTime);
}

void MPCController::cal_B_d_and_B_d_d()
{
    // Calculate _B_d and Calculate _B_d_d

    int select_count = 0;
    int num = 0;

    if (_LegContactState.LegState[0])
    {
        num = select_count * 3;

        _B_d(0, num) = 1 / 2 * (_R1(0, 0) * SamplingTime * SamplingTime * _Rz(0, 0) + _R1(1, 0) * SamplingTime * SamplingTime * _Rz(0, 1) + _R1(2, 0) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num) = 1 / 2 * (_R1(0, 0) * SamplingTime * SamplingTime * _Rz(1, 0) + _R1(1, 0) * SamplingTime * SamplingTime * _Rz(1, 1) + _R1(2, 0) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num) = 1 / 2 * (_R1(0, 0) * SamplingTime * SamplingTime * _Rz(2, 0) + _R1(1, 0) * SamplingTime * SamplingTime * _Rz(2, 1) + _R1(2, 0) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num + 1) = 1 / 2 * (_R1(0, 1) * SamplingTime * SamplingTime * _Rz(0, 0) + _R1(1, 1) * SamplingTime * SamplingTime * _Rz(0, 1) + _R1(2, 1) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num + 1) = 1 / 2 * (_R1(0, 1) * SamplingTime * SamplingTime * _Rz(1, 0) + _R1(1, 1) * SamplingTime * SamplingTime * _Rz(1, 1) + _R1(2, 1) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num + 1) = 1 / 2 * (_R1(0, 1) * SamplingTime * SamplingTime * _Rz(2, 0) + _R1(1, 1) * SamplingTime * SamplingTime * _Rz(2, 1) + _R1(2, 1) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num + 2) = 1 / 2 * (_R1(0, 2) * SamplingTime * SamplingTime * _Rz(0, 0) + _R1(1, 2) * SamplingTime * SamplingTime * _Rz(0, 1) + _R1(2, 2) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num + 2) = 1 / 2 * (_R1(0, 2) * SamplingTime * SamplingTime * _Rz(1, 0) + _R1(1, 2) * SamplingTime * SamplingTime * _Rz(1, 1) + _R1(2, 2) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num + 2) = 1 / 2 * (_R1(0, 2) * SamplingTime * SamplingTime * _Rz(2, 0) + _R1(1, 2) * SamplingTime * SamplingTime * _Rz(2, 1) + _R1(2, 2) * SamplingTime * SamplingTime * _Rz(2, 2));

        _B_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d.block<3, 3>(6, num) = _R1 * SamplingTime;
        _B_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        _B_d_d(0, num) = 1 / 2 * (_R1_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R1_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R1_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num) = 1 / 2 * (_R1_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R1_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R1_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num) = 1 / 2 * (_R1_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R1_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R1_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num + 1) = 1 / 2 * (_R1_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R1_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R1_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num + 1) = 1 / 2 * (_R1_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R1_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R1_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num + 1) = 1 / 2 * (_R1_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R1_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R1_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num + 2) = 1 / 2 * (_R1_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R1_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R1_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num + 2) = 1 / 2 * (_R1_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R1_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R1_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num + 2) = 1 / 2 * (_R1_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R1_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R1_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(2, 2));

        _B_d_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d_d.block<3, 3>(6, num) = _R1 * SamplingTime;
        _B_d_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        select_count++;
    }

    if (_LegContactState.LegState[1])
    {
        num = select_count * 3;

        _B_d(0, num) = 1 / 2 * (_R2(0, 0) * SamplingTime * SamplingTime * _Rz(0, 0) + _R2(1, 0) * SamplingTime * SamplingTime * _Rz(0, 1) + _R2(2, 0) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num) = 1 / 2 * (_R2(0, 0) * SamplingTime * SamplingTime * _Rz(1, 0) + _R2(1, 0) * SamplingTime * SamplingTime * _Rz(1, 1) + _R2(2, 0) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num) = 1 / 2 * (_R2(0, 0) * SamplingTime * SamplingTime * _Rz(2, 0) + _R2(1, 0) * SamplingTime * SamplingTime * _Rz(2, 1) + _R2(2, 0) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num + 1) = 1 / 2 * (_R2(0, 1) * SamplingTime * SamplingTime * _Rz(0, 0) + _R2(1, 1) * SamplingTime * SamplingTime * _Rz(0, 1) + _R2(2, 1) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num + 1) = 1 / 2 * (_R2(0, 1) * SamplingTime * SamplingTime * _Rz(1, 0) + _R2(1, 1) * SamplingTime * SamplingTime * _Rz(1, 1) + _R2(2, 1) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num + 1) = 1 / 2 * (_R2(0, 1) * SamplingTime * SamplingTime * _Rz(2, 0) + _R2(1, 1) * SamplingTime * SamplingTime * _Rz(2, 1) + _R2(2, 1) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num + 2) = 1 / 2 * (_R2(0, 2) * SamplingTime * SamplingTime * _Rz(0, 0) + _R2(1, 2) * SamplingTime * SamplingTime * _Rz(0, 1) + _R2(2, 2) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num + 2) = 1 / 2 * (_R2(0, 2) * SamplingTime * SamplingTime * _Rz(1, 0) + _R2(1, 2) * SamplingTime * SamplingTime * _Rz(1, 1) + _R2(2, 2) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num + 2) = 1 / 2 * (_R2(0, 2) * SamplingTime * SamplingTime * _Rz(2, 0) + _R2(1, 2) * SamplingTime * SamplingTime * _Rz(2, 1) + _R2(2, 2) * SamplingTime * SamplingTime * _Rz(2, 2));

        _B_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d.block<3, 3>(6, num) = _R2 * SamplingTime;
        _B_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        _B_d_d(0, num) = 1 / 2 * (_R2_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R2_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R2_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num) = 1 / 2 * (_R2_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R2_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R2_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num) = 1 / 2 * (_R2_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R2_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R2_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num + 1) = 1 / 2 * (_R2_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R2_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R2_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num + 1) = 1 / 2 * (_R2_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R2_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R2_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num + 1) = 1 / 2 * (_R2_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R2_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R2_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num + 2) = 1 / 2 * (_R2_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R2_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R2_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num + 2) = 1 / 2 * (_R2_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R2_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R2_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num + 2) = 1 / 2 * (_R2_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R2_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R2_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(2, 2));

        _B_d_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d_d.block<3, 3>(6, num) = _R2 * SamplingTime;
        _B_d_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        select_count++;
    }

    if (_LegContactState.LegState[2])
    {
        num = select_count * 3;

        _B_d(0, num) = 1 / 2 * (_R3(0, 0) * SamplingTime * SamplingTime * _Rz(0, 0) + _R3(1, 0) * SamplingTime * SamplingTime * _Rz(0, 1) + _R3(2, 0) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num) = 1 / 2 * (_R3(0, 0) * SamplingTime * SamplingTime * _Rz(1, 0) + _R3(1, 0) * SamplingTime * SamplingTime * _Rz(1, 1) + _R3(2, 0) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num) = 1 / 2 * (_R3(0, 0) * SamplingTime * SamplingTime * _Rz(2, 0) + _R3(1, 0) * SamplingTime * SamplingTime * _Rz(2, 1) + _R3(2, 0) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num+1) = 1 / 2 * (_R3(0, 1) * SamplingTime * SamplingTime * _Rz(0, 0) + _R3(1, 1) * SamplingTime * SamplingTime * _Rz(0, 1) + _R3(2, 1) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num+1) = 1 / 2 * (_R3(0, 1) * SamplingTime * SamplingTime * _Rz(1, 0) + _R3(1, 1) * SamplingTime * SamplingTime * _Rz(1, 1) + _R3(2, 1) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num+1) = 1 / 2 * (_R3(0, 1) * SamplingTime * SamplingTime * _Rz(2, 0) + _R3(1, 1) * SamplingTime * SamplingTime * _Rz(2, 1) + _R3(2, 1) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num+2) = 1 / 2 * (_R3(0, 2) * SamplingTime * SamplingTime * _Rz(0, 0) + _R3(1, 2) * SamplingTime * SamplingTime * _Rz(0, 1) + _R3(2, 2) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num+2) = 1 / 2 * (_R3(0, 2) * SamplingTime * SamplingTime * _Rz(1, 0) + _R3(1, 2) * SamplingTime * SamplingTime * _Rz(1, 1) + _R3(2, 2) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num+2) = 1 / 2 * (_R3(0, 2) * SamplingTime * SamplingTime * _Rz(2, 0) + _R3(1, 2) * SamplingTime * SamplingTime * _Rz(2, 1) + _R3(2, 2) * SamplingTime * SamplingTime * _Rz(2, 2));

        _B_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d.block<3, 3>(6, num) = _R3 * SamplingTime;
        _B_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        _B_d_d(0, num) = 1 / 2 * (_R3_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R3_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R3_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num) = 1 / 2 * (_R3_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R3_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R3_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num) = 1 / 2 * (_R3_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R3_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R3_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num+1) = 1 / 2 * (_R3_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R3_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R3_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num+1) = 1 / 2 * (_R3_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R3_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R3_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num+1) = 1 / 2 * (_R3_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R3_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R3_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num+2) = 1 / 2 * (_R3_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R3_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R3_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num+2) = 1 / 2 * (_R3_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R3_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R3_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num+2) = 1 / 2 * (_R3_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R3_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R3_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(2, 2));

        _B_d_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d_d.block<3, 3>(6, num) = _R3 * SamplingTime;
        _B_d_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        select_count++;
    }

    if (_LegContactState.LegState[3])
    {
        num = select_count * 3;

        _B_d(0, num) = 1 / 2 * (_R4(0, 0) * SamplingTime * SamplingTime * _Rz(0, 0) + _R4(1, 0) * SamplingTime * SamplingTime * _Rz(0, 1) + _R4(2, 0) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num) = 1 / 2 * (_R4(0, 0) * SamplingTime * SamplingTime * _Rz(1, 0) + _R4(1, 0) * SamplingTime * SamplingTime * _Rz(1, 1) + _R4(2, 0) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num) = 1 / 2 * (_R4(0, 0) * SamplingTime * SamplingTime * _Rz(2, 0) + _R4(1, 0) * SamplingTime * SamplingTime * _Rz(2, 1) + _R4(2, 0) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num + 1) = 1 / 2 * (_R4(0, 1) * SamplingTime * SamplingTime * _Rz(0, 0) + _R4(1, 1) * SamplingTime * SamplingTime * _Rz(0, 1) + _R4(2, 1) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num + 1) = 1 / 2 * (_R4(0, 1) * SamplingTime * SamplingTime * _Rz(1, 0) + _R4(1, 1) * SamplingTime * SamplingTime * _Rz(1, 1) + _R4(2, 1) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num + 1) = 1 / 2 * (_R4(0, 1) * SamplingTime * SamplingTime * _Rz(2, 0) + _R4(1, 1) * SamplingTime * SamplingTime * _Rz(2, 1) + _R4(2, 1) * SamplingTime * SamplingTime * _Rz(2, 2));
        _B_d(0, num + 2) = 1 / 2 * (_R4(0, 2) * SamplingTime * SamplingTime * _Rz(0, 0) + _R4(1, 2) * SamplingTime * SamplingTime * _Rz(0, 1) + _R4(2, 2) * SamplingTime * SamplingTime * _Rz(0, 2));
        _B_d(1, num + 2) = 1 / 2 * (_R4(0, 2) * SamplingTime * SamplingTime * _Rz(1, 0) + _R4(1, 2) * SamplingTime * SamplingTime * _Rz(1, 1) + _R4(2, 2) * SamplingTime * SamplingTime * _Rz(1, 2));
        _B_d(2, num + 2) = 1 / 2 * (_R4(0, 2) * SamplingTime * SamplingTime * _Rz(2, 0) + _R4(1, 2) * SamplingTime * SamplingTime * _Rz(2, 1) + _R4(2, 2) * SamplingTime * SamplingTime * _Rz(2, 2));

        _B_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d.block<3, 3>(6, num) = _R4 * SamplingTime;
        _B_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        _B_d_d(0, num) = 1 / 2 * (_R4_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R4_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R4_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num) = 1 / 2 * (_R4_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R4_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R4_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num) = 1 / 2 * (_R4_d(0, 0) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R4_d(1, 0) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R4_d(2, 0) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num + 1) = 1 / 2 * (_R4_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R4_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R4_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num + 1) = 1 / 2 * (_R4_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R4_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R4_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num + 1) = 1 / 2 * (_R4_d(0, 1) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R4_d(1, 1) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R4_d(2, 1) * SamplingTime * SamplingTime * _Rz_d(2, 2));
        _B_d_d(0, num + 2) = 1 / 2 * (_R4_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(0, 0) + _R4_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(0, 1) + _R4_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(0, 2));
        _B_d_d(1, num + 2) = 1 / 2 * (_R4_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(1, 0) + _R4_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(1, 1) + _R4_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(1, 2));
        _B_d_d(2, num + 2) = 1 / 2 * (_R4_d(0, 2) * SamplingTime * SamplingTime * _Rz_d(2, 0) + _R4_d(1, 2) * SamplingTime * SamplingTime * _Rz_d(2, 1) + _R4_d(2, 2) * SamplingTime * SamplingTime * _Rz_d(2, 2));

        _B_d_d.block<3, 3>(3, num) = _I3x3 * (SamplingTime * SamplingTime / (2 * _m_body));
        _B_d_d.block<3, 3>(6, num) = _R4 * SamplingTime;
        _B_d_d.block<3, 3>(9, num) = _I3x3 * (SamplingTime / _m_body);

        select_count++;
    }
}