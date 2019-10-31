#include <legged_robot/quadruped_robot.h>


QuadrupedRobot::QuadrupedRobot()
{
  // command and state divided into each legs (4x3)
  for (int i=0; i<4; i++)
  {
    // joint state
    _q_leg_kdl[i].resize(3);
    _qdot_leg_kdl[i].resize(3);

    // Jacobian
    _J_leg[i].resize(3);

    // Dynamics
    _M_leg[i].resize(3);
    _C_leg[i].resize(3);
    _G_leg[i].resize(3);
  }
}

int QuadrupedRobot::init()
{
  // kdl chain
  std::string root_name, tip_name[4];

  root_name = "trunk";
  tip_name[0] = "lf_foot"; tip_name[1] = "rf_foot"; tip_name[2] = "lh_foot"; tip_name[3] = "rh_foot";

  _gravity.reset(new KDL::Vector(0.0, 0.0, -9.81));

  for (int i=0; i<4; i++)
  {
    if(!_kdl_tree.getChain(root_name, tip_name[i], _kdl_chain[i]))
    {
      return -1;
    }
    else
    {
      _fk_pos_solver[i].reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain[i]));
      _jnt_to_jac_solver[i].reset(new KDL::ChainJntToJacSolver(_kdl_chain[i]));
      // _jnt_to_jac_dot_solver[i].reset(new KDL::ChainJntToJacDotSolver(_kdl_chain[i])); @ To do
      _id_solver[i].reset(new KDL::ChainDynParam(_kdl_chain[i],*_gravity));
    }
  }
}

void QuadrupedRobot::updateSensorData(const std::array<Eigen::Vector3d, 4>& q_leg, const std::array<Eigen::Vector3d, 4>& qdot_leg,
                        const Pose& pose_body, const PoseVel& pose_vel_body)
{
  _q_leg = q_leg;
  _qdot_leg = qdot_leg;
  for (int i=0; i<4; i++)
  {
    for (int j=0; j<3; j++)
    {
      _q_leg_kdl[i](j) = _q_leg[i][j];
      _qdot_leg_kdl[i](j) = _qdot_leg[i][j];
    }
  }
  _pose_body = pose_body;
  _pose_vel_body = pose_vel_body;

}

void QuadrupedRobot::calKinematics()
{
  std::array<KDL::Frame,4> frame_leg;

  for (size_t i=0; i<4; i++)
  {
    _fk_pos_solver[i]->JntToCart(_q_leg_kdl[i], frame_leg[i]);

    _jnt_to_jac_solver[i]->JntToJac(_q_leg_kdl[i], _J_leg[i]);
    _Jv_leg[i] = _J_leg[i].data.block(0,0,3,3);
    _p_body2leg[i] = Eigen::Vector3d(frame_leg[i].p.data);
    _v_body2leg[i] = _Jv_leg[i]*_qdot_leg_kdl[i].data;

    _q_leg[i] = Eigen::Vector3d(frame_leg[i].p.data);
    _qdot_leg[i] = _Jv_leg[i]*_qdot_leg_kdl[i].data;

    // _jnt_to_jac_dot_solver[i]->JntToJacDot(); // @ To do: Jacobian Dot Calculation

    _id_solver[i]->JntToMass(_q_leg_kdl[i], _M_leg[i]);
    _id_solver[i]->JntToCoriolis(_q_leg_kdl[i], _qdot_leg_kdl[i], _C_leg[i]);
    _id_solver[i]->JntToGravity(_q_leg_kdl[i], _G_leg[i]);
  }

  // to world coordinates
  for (int i=0; i<4; i++)
      _p_world2leg[i] = _pose_body * _p_body2leg[i];

  // body to com
  _pose_com_d._pos = _pose_body_d * _p_body2com;
  _pose_com_d._rot_quat = _pose_body_d._rot_quat;

  _pose_com._pos = _pose_body * _p_body2com;
  _pose_com._rot_quat = _pose_body._rot_quat;

  _pose_vel_com_d._linear = _pose_vel_body_d._linear + skew(_pose_vel_body_d._linear) * _pose_body_d._rot_quat.toRotationMatrix() * _p_body2com;
  _pose_vel_com_d._angular = _pose_vel_body_d._angular;
}
