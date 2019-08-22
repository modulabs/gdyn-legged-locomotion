#include <legged_controllers/leg_controller.h>

using namespace legged_controllers;

namespace legged_controllers
{

bool LegController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{	
	_loop_count = 0;
	_node_ptr = &n;

	// List of controlled joints
	if (!n.getParam("joints", _joint_names))
	{
		ROS_ERROR("Could not find joint name");
		return false;
	}
	_n_joints = _joint_names.size();

	if(_n_joints == 0)
	{
		ROS_ERROR("List of joint names is empty.");
		return false;
	}
	else
	{
		ROS_INFO("Find %d joints", _n_joints);
	}

	// urdf
	urdf::Model urdf;
	if (!urdf.initParam("robot_description"))
	{
		ROS_ERROR("Failed to parse urdf file");
		return false;
	}

	// joint handle
	for(int i=0; i<_n_joints; i++)
	{
		try
		{
			_joints.push_back(hw->getHandle(_joint_names[i]));
		}
		catch (const hardware_interface::HardwareInterfaceException& e)
		{
			ROS_ERROR_STREAM("Exception thrown: " << e.what());
			return false;
		}

		urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(_joint_names[i]);
		if (!joint_urdf)
		{
			ROS_ERROR("Could not find joint '%s' in urdf", _joint_names[i].c_str());
			return false;
		}
		_joint_urdfs.push_back(joint_urdf); 
	}

	// kdl parser
	if (!kdl_parser::treeFromUrdfModel(urdf, _kdl_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	// kdl chain
	std::string root_name, tip_name[4];
	root_name = "trunk";
	tip_name[0] = "lf_foot"; tip_name[1] = "rf_foot"; tip_name[2] = "lh_foot"; tip_name[3] = "rh_foot";
	for (int i=0; i<4; i++)
	{
		if(!_kdl_tree.getChain(root_name, tip_name[i], _kdl_chain[i]))
		{
			ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
			ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);

			return false;
		}
		else
		{
			_fk_solver[i].reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain[i]));
		}
	}
	

	// command and state
	_tau_d.data = Eigen::VectorXd::Zero(_n_joints);
	_tau_fric.data = Eigen::VectorXd::Zero(_n_joints);
	_q_d.data = Eigen::VectorXd::Zero(_n_joints);
	_qdot_d.data = Eigen::VectorXd::Zero(_n_joints);
	_qddot_d.data = Eigen::VectorXd::Zero(_n_joints);
	_q_d_old.data = Eigen::VectorXd::Zero(_n_joints);
	_qdot_d_old.data = Eigen::VectorXd::Zero(_n_joints);
	
	_q.data = Eigen::VectorXd::Zero(_n_joints);
	_qdot.data = Eigen::VectorXd::Zero(_n_joints);

	_q_error.data = Eigen::VectorXd::Zero(_n_joints);
	_qdot_error.data = Eigen::VectorXd::Zero(_n_joints);

	// gains
	_kp.resize(_n_joints);
	_kd.resize(_n_joints);

	// service updating gain
	_gains_kp_buffer.writeFromNonRT(std::vector<double>(_n_joints, 0.0));
	_gains_kd_buffer.writeFromNonRT(std::vector<double>(_n_joints, 0.0));
	_update_gain_srv = n.advertiseService("update_gain", &LegController::updateGain, this);

	updateGain();

	// command subscriber
	_commands_buffer.writeFromNonRT(std::vector<double>(_n_joints, 0.0));
	_command_sub = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &LegController::setCommand, this);

	// start realtime state publisher
	_controller_state_pub.reset(
		new realtime_tools::RealtimePublisher<legged_controllers::ControllerJointState>(n, "state", 1));

	_controller_state_pub->msg_.header.stamp = ros::Time::now();
	for (size_t i=0; i<_n_joints; i++)
	{
		_controller_state_pub->msg_.name.push_back(_joint_names[i]);
		_controller_state_pub->msg_.command.push_back(0.0);
		_controller_state_pub->msg_.command_dot.push_back(0.0);
		_controller_state_pub->msg_.state.push_back(0.0);
		_controller_state_pub->msg_.state_dot.push_back(0.0);
		_controller_state_pub->msg_.q_error.push_back(0.0);
		_controller_state_pub->msg_.qdot_error.push_back(0.0);
		_controller_state_pub->msg_.effort_command.push_back(0.0);
		_controller_state_pub->msg_.effort_feedforward.push_back(0.0);
		_controller_state_pub->msg_.effort_feedback.push_back(0.0);
	}

	// thread: balance controller
	double m_body, mu;
	KDL::RotationalInertia I_com;
	_balance_controller.init(m_body, I_com, mu);
	
	return true;
}

void LegController::starting(const ros::Time& time)
{
	// get joint positions
	for(size_t i=0; i<_n_joints; i++) 
	{
		_q(i) = _joints[i].getPosition();
		_qdot(i) = _joints[i].getVelocity();
	}

	ROS_INFO("Starting Leg Controller");
}

void LegController::setCommand(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	if(msg->data.size()!=_n_joints)
	{ 
	ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << _n_joints << ")! Not executing!");
	return; 
	}
	_commands_buffer.writeFromNonRT(msg->data);
}

bool LegController::updateGain(UpdateGain::Request& request, UpdateGain::Response& response)
{
	updateGain();
}

bool LegController::updateGain()
{
	std::vector<double> kp(_n_joints), kd(_n_joints);
	std::string gain_name;
	for (size_t i = 0; i < _n_joints; i++)
	{
		// FIXME. generalize name of robot
		gain_name = "/hyq/leg_controller/gains/" + _joint_names[i] + "/p";
		if (_node_ptr->getParam(gain_name, kp[i]))
		{
			ROS_INFO("Updage gain %s = %.2f", gain_name.c_str(), kp[i]);
		}
		else
		{
			ROS_ERROR("Cannot find %s gain", gain_name.c_str());
			return false;
		}

		gain_name = "/hyq/leg_controller/gains/" + _joint_names[i] + "/d";
		if (_node_ptr->getParam(gain_name, kd[i]))
		{
			ROS_INFO("Updage gain %s = %.2f", gain_name.c_str(), kd[i]);
		}
		else
		{
			ROS_ERROR("Cannot find %s gain", gain_name.c_str());
			return false;
		}
	}

	_gains_kp_buffer.writeFromNonRT(kp);
	_gains_kd_buffer.writeFromNonRT(kd);

	return true;
}

void LegController::update(const ros::Time& time, const ros::Duration& period)
{
	std::vector<double> & commands = *_commands_buffer.readFromRT();
	std::vector<double> & kp = *_gains_kp_buffer.readFromRT();
	std::vector<double> & kd = *_gains_kd_buffer.readFromRT();
	double dt = period.toSec();
	double q_d_old;

	// update gains and joint commands/states
	static double t = 0;
	for (size_t i=0; i<_n_joints; i++)
	{
		// gains
		_kp(i) = kp[i];
		_kd(i) = kd[i];
	}

	// update state
	KDL::Vector p_com;

	// update trajectory
	KDL::Vector p_com_d, p_com_dot_d, w_body_d, w_body;
	KDL::Rotation R_body_d, R_body;

	// forward kinematics
	std::array<KDL::Frame,4> frame_leg;
	for (size_t i=0; i<4; i++)
	{
		_fk_solver[i]->JntToCart(_q_leg[i], frame_leg[i]);
		_p_leg[i] = frame_leg[i].p;
	}	

	// balance controller
	_balance_controller.setControlInput(p_com_d, p_com_dot_d, R_body_d, w_body_d,
							p_com, _p_leg, R_body, w_body);

	_balance_controller.update();

	_balance_controller.getControlOutput(_F_leg);		

	// force to torque
	for (size_t i=0; i<4; i++)
	{
		// _tau_d = J[i].transpose() * _F_leg[i];	// position jacobian transpose mapping
	}

	// torque command
	for(int i=0; i<_n_joints; i++)
	{
		_tau_d(i) = _kp(i)*_q_error(i) + _kd(i)*_qdot_error(i);

		// effort saturation
		if (_tau_d(i) >= _joint_urdfs[i]->limits->effort)
			_tau_d(i) = _joint_urdfs[i]->limits->effort;
		
		if (_tau_d(i) <= -_joint_urdfs[i]->limits->effort)
			_tau_d(i) = -_joint_urdfs[i]->limits->effort;

		_joints[i].setCommand( _tau_d(i) );
	}

	// publish
	if (_loop_count % 10 == 0)
	{
		if (_controller_state_pub->trylock())
		{
			_controller_state_pub->msg_.header.stamp = time;
			for(int i=0; i<_n_joints; i++)
			{
				_controller_state_pub->msg_.command[i] = R2D*_q_d(i);
				_controller_state_pub->msg_.command_dot[i] = R2D*_qdot_d(i);
				_controller_state_pub->msg_.state[i] = R2D*_q(i);
				_controller_state_pub->msg_.state_dot[i] = R2D*_qdot(i);
				_controller_state_pub->msg_.q_error[i] = R2D*_q_error(i);
				_controller_state_pub->msg_.qdot_error[i] = R2D*_qdot_error(i);
				_controller_state_pub->msg_.effort_command[i] = _tau_d(i);
				_controller_state_pub->msg_.effort_feedback[i] = _tau_d(i) - _controller_state_pub->msg_.effort_feedforward[i];
			}
			_controller_state_pub->unlockAndPublish();
		}
	}
}

void LegController::enforceJointLimits(double &command, unsigned int index)
{
	// Check that this joint has applicable limits
	if (_joint_urdfs[index]->type == urdf::Joint::REVOLUTE || _joint_urdfs[index]->type == urdf::Joint::PRISMATIC)
	{
	if( command > _joint_urdfs[index]->limits->upper ) // above upper limnit
	{
		command = _joint_urdfs[index]->limits->upper;
	}
	else if( command < _joint_urdfs[index]->limits->lower ) // below lower limit
	{
		command = _joint_urdfs[index]->limits->lower;
	}
	}
}

}
PLUGINLIB_EXPORT_CLASS(legged_controllers::LegController, controller_interface::ControllerBase)

