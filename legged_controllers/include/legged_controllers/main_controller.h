#pragma once

#include <array>
#include <boost/scoped_ptr.hpp>

// ros control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/PoseArray.h>
#include <legged_controllers/MoveBody.h>

// kdl
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics: position
#include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics: velocity
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
// #include <kdl/chainjnttojacdotsolver.hpp>     // @ To do: jacobian derivative
#include <kdl/chaindynparam.hpp>              // inverse dynamics


#include <legged_controllers/ControllerJointState.h>
#include <legged_controllers/UpdateGain.h>

#include <legged_robot/quadruped_robot.h>

#include <legged_controllers/balance_controller.h>
#include <legged_controllers/virtaul_spring_damper_controller.h>
#include <legged_controllers/swing_controller.h>
#include <legged_controllers/mpc_controller.h>

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace legged_controllers
{

class Trunk
{
public:
	Eigen::Vector3d _p, _v;	// position, linear velocity
	Eigen::Quaterniond _o;	// orientation
	Eigen::Vector3d _w;		// angular velocity
};

class MainController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
	~MainController() { _commands_sub.shutdown(); _link_states_sub.shutdown();}

	bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
	
	void starting(const ros::Time& time);
	void stopping(const ros::Time& time) { }

	// subscribe
	void subscribeCommand(const std_msgs::Float64MultiArrayConstPtr& msg);
	void subscribeTrunkState(const gazebo_msgs::LinkStatesConstPtr& msg);
  void subscribeLFContactState(const gazebo_msgs::ContactsStateConstPtr& msg);
  void subscribeRFContactState(const gazebo_msgs::ContactsStateConstPtr& msg);
  void subscribeLHContactState(const gazebo_msgs::ContactsStateConstPtr& msg);
  void subscribeRHContactState(const gazebo_msgs::ContactsStateConstPtr& msg);

	// service
	bool srvMoveBodyCB(MoveBody::Request& request, MoveBody::Response& response);

	bool updateGain(UpdateGain::Request& request, UpdateGain::Response& response);
	bool updateGain();

	void update(const ros::Time& time, const ros::Duration& period);

	void enforceJointLimits(double &command, unsigned int index);
	void print_state();

private:
	int _loop_count;
	double _t;

	// ros nodehandle
	ros::NodeHandle* _node_ptr;

	// joint handles
	unsigned int _n_joints;
	std::vector<std::string> _joint_names;
	std::vector<hardware_interface::JointHandle> _joints;
	std::vector<urdf::JointConstSharedPtr> _joint_urdfs;



  // Quadruped Robot
  quadruped_robot::QuadrupedRobot _robot;

	// cmd, state
	realtime_tools::RealtimeBuffer<std::vector<double> > _commands_buffer;
	realtime_tools::RealtimeBuffer<std::vector<double> > _gains_kp_buffer;
	realtime_tools::RealtimeBuffer<std::vector<double> > _gains_kd_buffer;

	realtime_tools::RealtimeBuffer<Trunk> _trunk_state_buffer;
  std::array<realtime_tools::RealtimeBuffer<bool>, 4> _contact_states_buffer;

	//
	KDL::JntArray _tau_d, _tau_fric;
  KDL::JntArray _qdot_d, _qddot_d, _q_d_old, _qdot_d_old;
	KDL::JntArray _q_error, _qdot_error;

	//

	std::array<Eigen::Vector3d, 4> _F_leg;
	std::array<Eigen::Vector3d, 4> _F_leg_balance; 	// FIXME. temporary
	std::array<Eigen::Vector3d, 4> _tau_leg;


	// 
	BalanceController _balance_controller;
	VirtualSpringDamperController _virtual_spring_damper_controller;
	MPCController _mpc_controller;

	// gain
	KDL::JntArray _kp, _kd;

	// topic
  ros::Subscriber _commands_sub, _link_states_sub;
  std::array<ros::Subscriber, 4> _contact_states_sub;
	boost::scoped_ptr<
		realtime_tools::RealtimePublisher<
			legged_controllers::ControllerJointState> > _controller_state_pub;
	
	// service
	ros::ServiceServer _update_gain_srv;
};

}


