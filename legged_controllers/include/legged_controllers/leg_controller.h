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

// kdl
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <legged_controllers/ControllerJointState.h>
#include <legged_controllers/UpdateGain.h>

#include <legged_controllers/balance_controller.h>

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace legged_controllers
{

class LegController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
	~LegController() { _command_sub.shutdown(); }

	bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
	
	void starting(const ros::Time& time);
	void stopping(const ros::Time& time) { }

	void setCommand(const std_msgs::Float64MultiArrayConstPtr& msg);
	bool updateGain(UpdateGain::Request& request, UpdateGain::Response& response);
	bool updateGain();

	void update(const ros::Time& time, const ros::Duration& period);

	void enforceJointLimits(double &command, unsigned int index);

private:
	int _loop_count;

	// ros nodehandle
	ros::NodeHandle* _node_ptr;

	// joint handles
	unsigned int _n_joints;
	std::vector<std::string> _joint_names;
	std::vector<hardware_interface::JointHandle> _joints;
	std::vector<urdf::JointConstSharedPtr> _joint_urdfs;

	// kdl
	KDL::Tree 	_kdl_tree;
	std::array<KDL::Chain, 4>	_kdl_chain;
	std::array<boost::scoped_ptr<KDL::ChainFkSolverPos_recursive>, 4> _fk_solver;

	// cmd, state
	realtime_tools::RealtimeBuffer<std::vector<double> > _commands_buffer;
	realtime_tools::RealtimeBuffer<std::vector<double> > _gains_kp_buffer;
	realtime_tools::RealtimeBuffer<std::vector<double> > _gains_kd_buffer;


	std::array<KDL::JntArray,4> _q_leg;
	KDL::JntArray _tau_d, _tau_fric;
	KDL::JntArray _q_d, _qdot_d, _qddot_d, _q_d_old, _qdot_d_old;
	KDL::JntArray _q, _qdot;
	KDL::JntArray _q_error, _qdot_error;

	//
	std::array<KDL::Vector, 4> _p_leg;
	std::array<KDL::Vector, 4> _F_leg;

	// 
	BalanceController _balance_controller;

	// gain
	KDL::JntArray _kp, _kd;

	// topic
	ros::Subscriber _command_sub;
	boost::scoped_ptr<
		realtime_tools::RealtimePublisher<
			legged_controllers::ControllerJointState> > _controller_state_pub;
	
	// service
	ros::ServiceServer _update_gain_srv;
};

}


