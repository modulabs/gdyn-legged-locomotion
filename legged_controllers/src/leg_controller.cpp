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

	_gravity.reset(new KDL::Vector(0.0, 0.0, -9.81));

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
			_fk_pos_solver[i].reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain[i]));
			_jnt_to_jac_solver[i].reset(new KDL::ChainJntToJacSolver(_kdl_chain[i]));
			// _jnt_to_jac_dot_solver[i].reset(new KDL::ChainJntToJacDotSolver(_kdl_chain[i])); @ To do
			_id_solver[i].reset(new KDL::ChainDynParam(_kdl_chain[i],*_gravity));
		}
	}
	

	// command and state (12x1)
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

    // command and state divided into each legs (4x3)
	for (int i=0; i<4; i++)
	{
        // joint state
        _q_leg[i].resize(3);
		_qdot_leg[i].resize(3);

		// Jacobian
		_J_leg[i].resize(3);

		// Dynamics
		_M_leg[i].resize(3);
		_C_leg[i].resize(3);
		_G_leg[i].resize(3);
    }

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
	_commands_sub = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &LegController::setCommand, this);

	// state subscriber
	_states_buffer.writeFromNonRT(gazebo_msgs::LinkStates());
	_states_sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo_msgs/link_states", 1, &LegController::setStates, this);


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

	// virtaul spring damper controller
	_virtual_spring_damper_controller.init();

	// balance controller
	double m_body = 60, mu = 0.6;	// TO DO: get this value from robot model
	Eigen::Matrix3d I_com = Eigen::Matrix3d::Zero();
	I_com.diagonal() << 1.5725937, 8.5015928, 9.1954911;
	Eigen::Vector3d p_body2com(0.056, 0.0215, 0.00358);
	_balance_controller.init(m_body, p_body2com, I_com, mu);
	
	return true;
}

void LegController::starting(const ros::Time& time)
{
	_t = 0;
	
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

void LegController::setStates(const gazebo_msgs::LinkStatesConstPtr& msg)
{
	_states_buffer.writeFromNonRT(*msg);
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
			ROS_INFO("Update gain %s = %.2f", gain_name.c_str(), kp[i]);
		}
		else
		{
			ROS_ERROR("Cannot find %s gain", gain_name.c_str());
			return false;
		}

		gain_name = "/hyq/leg_controller/gains/" + _joint_names[i] + "/d";
		if (_node_ptr->getParam(gain_name, kd[i]))
		{
			ROS_INFO("Update gain %s = %.2f", gain_name.c_str(), kd[i]);
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
	gazebo_msgs::LinkStates& link_states = *_states_buffer.readFromRT();

	double dt = period.toSec();
	double q_d_old;

	_t += dt;


	// update gains and joint commands/states
	for (size_t i=0; i<_n_joints; i++)
	{
		// state
		_q(i) = _joints[i].getPosition();
		_qdot(i) = _joints[i].getVelocity();
		
		// gains
		_kp(i) = kp[i];
		_kd(i) = kd[i];
	}

	// update state from tree (12x1) to each leg (4x3)
	for (size_t i = 0; i < _n_joints; i++)
	{
		if (i < 3)
		{
			_q_leg[0](i) = _q(i);
			_qdot_leg[0](i) = _qdot(i);
		}
		else if (i < 6)
		{
			_q_leg[1](i - 3) = _q(i);
			_qdot_leg[1](i - 3) = _qdot(i);
		}
		else if (i < 9)
		{
			_q_leg[2](i - 6) = _q(i);
			_qdot_leg[2](i - 6) = _qdot(i);
		}
		else
		{
			_q_leg[3](i - 9) = _q(i);
			_qdot_leg[3](i - 9) = _qdot(i);
		}
	}

	// State - continuosly update, subscribe from gazebo for now, TODO: get this from state observer
	Eigen::Vector3d p_body, p_body_dot, w_body;
	Eigen::Matrix3d R_body;

	// p_body(0) = link_states.pose[1].position.x;
	// p_body(1) = link_states.pose[1].position.y;
	// p_body(2) = link_states.pose[1].position.z;

	// Eigen::Quaterniond qu(link_states.pose[1].orientation.w, 
	// 	link_states.pose[1].orientation.x, link_states.pose[1].orientation.y, link_states.pose[1].orientation.z);
	// R_body = qu.toRotationMatrix();

	// p_body_dot(0) = link_states.twist[1].linear.x;
	// p_body_dot(1) = link_states.twist[1].linear.y;
	// p_body_dot(2) = link_states.twist[1].linear.z;

	// w_body(0) = link_states.twist[1].angular.x;
	// w_body(1) = link_states.twist[1].angular.y;
	// w_body(2) = link_states.twist[1].angular.z;

	// update trajectory - get from this initial state(temporary)
	Eigen::Vector3d p_body_d, p_body_dot_d, w_body_d;
	Eigen::Matrix3d R_body_d;
	static int td = 0;

	if (td++ == 3000)
	{
		p_body_d = p_body;
	}
	p_body_dot_d.setZero();

	R_body_d.setIdentity();
	w_body_d.setZero();


	// forward kinematics
	std::array<KDL::Frame,4> frame_leg;

	for (size_t i=0; i<4; i++)
	{
		_fk_pos_solver[i]->JntToCart(_q_leg[i], frame_leg[i]);

		_jnt_to_jac_solver[i]->JntToJac(_q_leg[i], _J_leg[i]);
		_Jv_leg[i] = _J_leg[i].data.block(0,0,3,3);	
		_p_leg[i] = Eigen::Vector3d(frame_leg[i].p.data);
		_v_leg[i] = _Jv_leg[i]*_qdot_leg[i].data;

		// _jnt_to_jac_dot_solver[i]->JntToJacDot(); // @ To do: Jacobian Dot Calculation

		_id_solver[i]->JntToMass(_q_leg[i], _M_leg[i]);
		_id_solver[i]->JntToCoriolis(_q_leg[i], _qdot_leg[i], _C_leg[i]);
        _id_solver[i]->JntToGravity(_q_leg[i], _G_leg[i]);
	}

	// // * v.01 - force calculation controller: Set force zero (_F_leg   -   Eigen::Vector3d)
	// for (size_t i=0; i<4; i++)
	// {
	// 	_F_leg[i].setZero(3);
	// }

	// * v.02 - force calculation controller: Set virtual spring force controller (_F_leg   -   Eigen::Vector3d)
	_virtual_spring_damper_controller.setControlInput(_p_leg,_v_leg,_G_leg);
	_virtual_spring_damper_controller.compute();
	_virtual_spring_damper_controller.getControlOutput(_F_leg);

	// * v.03 - force calculation controller: balance controller by MIT cheetah (_F_leg  -  Eigen::Vector3d)
	_balance_controller.setControlInput(p_body_d, p_body_dot_d, R_body_d, w_body_d,
							p_body, p_body_dot, R_body, w_body, _p_leg);
	_balance_controller.update();
	_balance_controller.getControlOutput(_F_leg_balance);
	if (td > 3000)
	{
		;
	}
	
	// convert force to torque	
	for (size_t i=0; i<4; i++)
	{
		_tau_leg[i] = _Jv_leg[i].transpose() * _F_leg[i];
		//_tau_leg[i] << 0, 0, 0;
	}

	// torque command
	for(int i=0; i<_n_joints; i++)
	{
		if (i < 3)
			_tau_d(i) = _tau_leg[0](i);
		else if (i < 6)
			_tau_d(i) = _tau_leg[1](i-3);
		else if (i < 9)
			_tau_d(i) = _tau_leg[2](i-6);
		else
			_tau_d(i) = _tau_leg[3](i-9);
	}

	for(int i=0; i<_n_joints; i++)
	{
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
				// FIXME. temporarily
				// _controller_state_pub->msg_.effort_command[i] = _tau_d(i);
				// _controller_state_pub->msg_.effort_feedback[i] = _tau_d(i) - _controller_state_pub->msg_.effort_feedforward[i];
			}

			for(int i=0; i<4; i++)
			{
				for(int j=0; j<3; j++)
				{
					_controller_state_pub->msg_.effort_command[i*3+j] = _F_leg[i](j);
					_controller_state_pub->msg_.effort_feedback[i*3+j] = _F_leg_balance[i](j);
				}
			}
			_controller_state_pub->unlockAndPublish();
		}
	}

    // ********* printf state *********
	print_state();
}

void LegController::enforceJointLimits(double &command, unsigned int index)
{
	// Check that this joint has applicable limits
	if (_joint_urdfs[index]->type == urdf::Joint::REVOLUTE || _joint_urdfs[index]->type == urdf::Joint::PRISMATIC)
	{
		if (command > _joint_urdfs[index]->limits->upper) // above upper limnit
		{
			command = _joint_urdfs[index]->limits->upper;
		}
		else if (command < _joint_urdfs[index]->limits->lower) // below lower limit
		{
			command = _joint_urdfs[index]->limits->lower;
		}
	}
}

void LegController::print_state()
{
	static int count = 0;
	if (count > 990)
	{
		printf("*********************************************************\n\n");
		printf("*** Simulation Time (unit: sec)  ***\n");
		printf("t = %f\n", _t);
		printf("\n");

		printf("*********************** Left Front Leg **********************************\n\n");

		printf("*** Actual Position in Joint Space (unit: deg) ***\n");
		printf("q(1): %f, ", _q_leg[0](0) * R2D);
		printf("q(2): %f, ", _q_leg[0](1) * R2D);
		printf("q(3): %f, ", _q_leg[0](2) * R2D);
		printf("\n");
		printf("\n");

		printf("*** Actual Position in Task Space (unit: m) ***\n");
		printf("x(1): %f, ", _p_leg[0](0) * 1);
		printf("x(2): %f, ", _p_leg[0](1) * 1);
		printf("x(3): %f, ", _p_leg[0](2) * 1);
		printf("\n");
		printf("\n");

		printf("*** Virtual Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg[0](0));
		printf("Y Force Input: %f, ", _F_leg[0](1));
		printf("Z Force Input: %f, ", _F_leg[0](2));
		printf("\n");
		printf("\n");

		printf("*** Balance Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg_balance[0](0));
		printf("Y Force Input: %f, ", _F_leg_balance[0](1));
		printf("Z Force Input: %f, ", _F_leg_balance[0](2));
		printf("\n");
		printf("\n");

		printf("*** Torque Input (unit: Nm) ***\n");
		printf("Hip AA Input: %f, ", _tau_leg[0](0));
		printf("Hip FE Input: %f, ", _tau_leg[0](1));
		printf("Knee FE Input: %f, ", _tau_leg[0](2));
		printf("\n");
		printf("\n");

		// printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
		// printf("Hip AA Input: %f, ", _G_leg[0](0));
		// printf("Hip FE Input: %f, ", _G_leg[0](1));
		// printf("Knee FE Input: %f, ", _G_leg[0](2));
		// printf("\n");
		// printf("\n");

		printf("*********************** Right Front Leg **********************************\n\n");
		printf("*** Actual Position in Joint Space (unit: deg) ***\n");
		printf("q(1): %f, ", _q_leg[1](0) * R2D);
		printf("q(2): %f, ", _q_leg[1](1) * R2D);
		printf("q(3): %f, ", _q_leg[1](2) * R2D);
		printf("\n");
		printf("\n");

		printf("*** Actual Position in Task Space (unit: m) ***\n");
		printf("x(1): %f, ", _p_leg[1](0) * 1);
		printf("x(2): %f, ", _p_leg[1](1) * 1);
		printf("x(3): %f, ", _p_leg[1](2) * 1);
		printf("\n");
		printf("\n");

		printf("*** Virtual Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg[1](0));
		printf("Y Force Input: %f, ", _F_leg[1](1));
		printf("Z Force Input: %f, ", _F_leg[1](2));
		printf("\n");
		printf("\n");

		printf("*** Balance Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg_balance[1](0));
		printf("Y Force Input: %f, ", _F_leg_balance[1](1));
		printf("Z Force Input: %f, ", _F_leg_balance[1](2));
		printf("\n");
		printf("\n");

		printf("*** Torque Input (unit: Nm) ***\n");
		printf("Hip AA Input: %f, ", _tau_leg[1](0));
		printf("Hip FE Input: %f, ", _tau_leg[1](1));
		printf("Knee FE Input: %f, ", _tau_leg[1](2));
		printf("\n");
		printf("\n");

		// printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
		// printf("Hip AA Input: %f, ", _G_leg[1](0));
		// printf("Hip FE Input: %f, ", _G_leg[1](1));
		// printf("Knee FE Input: %f, ", _G_leg[1](2));
		// printf("\n");
		// printf("\n");

		printf("*********************** Left Hind Leg **********************************\n\n");
		printf("*** Actual Position in Joint Space (unit: deg) ***\n");
		printf("q(1): %f, ", _q_leg[2](0) * R2D);
		printf("q(2): %f, ", _q_leg[2](1) * R2D);
		printf("q(3): %f, ", _q_leg[2](2) * R2D);
		printf("\n");
		printf("\n");

		printf("*** Actual Position in Task Space (unit: m) ***\n");
		printf("x(1): %f, ", _p_leg[2](0) * 1);
		printf("x(2): %f, ", _p_leg[2](1) * 1);
		printf("x(3): %f, ", _p_leg[2](2) * 1);
		printf("\n");
		printf("\n");

		printf("*** Virtual Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg[2](0));
		printf("Y Force Input: %f, ", _F_leg[2](1));
		printf("Z Force Input: %f, ", _F_leg[2](2));
		printf("\n");
		printf("\n");

		printf("*** Balance Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg_balance[2](0));
		printf("Y Force Input: %f, ", _F_leg_balance[2](1));
		printf("Z Force Input: %f, ", _F_leg_balance[2](2));
		printf("\n");
		printf("\n");

		printf("*** Torque Input (unit: Nm) ***\n");
		printf("Hip AA Input: %f, ", _tau_leg[2](0));
		printf("Hip FE Input: %f, ", _tau_leg[2](1));
		printf("Knee FE Input: %f, ", _tau_leg[2](2));
		printf("\n");
		printf("\n");

		// printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
		// printf("Hip AA Input: %f, ", _G_leg[2](0));
		// printf("Hip FE Input: %f, ", _G_leg[2](1));
		// printf("Knee FE Input: %f, ", _G_leg[2](2));
		// printf("\n");
		// printf("\n");

		printf("*********************** Right Hind Leg **********************************\n\n");
		printf("*** Actual Position in Joint Space (unit: deg) ***\n");
		printf("q(1): %f, ", _q_leg[3](0) * R2D);
		printf("q(2): %f, ", _q_leg[3](1) * R2D);
		printf("q(3): %f, ", _q_leg[3](2) * R2D);
		printf("\n");
		printf("\n");

		printf("*** Actual Position in Task Space (unit: m) ***\n");
		printf("x(1): %f, ", _p_leg[3](0) * 1);
		printf("x(2): %f, ", _p_leg[3](1) * 1);
		printf("x(3): %f, ", _p_leg[3](2) * 1);
		printf("\n");
		printf("\n");

		printf("*** Virtual Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg[3](0));
		printf("Y Force Input: %f, ", _F_leg[3](1));
		printf("Z Force Input: %f, ", _F_leg[3](2));
		printf("\n");
		printf("\n");

		printf("*** Balance Leg Forces (unit: N) ***\n");
		printf("X Force Input: %f, ", _F_leg_balance[3](0));
		printf("Y Force Input: %f, ", _F_leg_balance[3](1));
		printf("Z Force Input: %f, ", _F_leg_balance[3](2));
		printf("\n");
		printf("\n");

		printf("*** Torque Input (unit: Nm) ***\n");
		printf("Hip AA Input: %f, ", _tau_leg[3](0));
		printf("Hip FE Input: %f, ", _tau_leg[3](1));
		printf("Knee FE Input: %f, ", _tau_leg[3](2));
		printf("\n");
		printf("\n");

		// printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
		// printf("Hip AA Input: %f, ", _G_leg[3](0));
		// printf("Hip FE Input: %f, ", _G_leg[3](1));
		// printf("Knee FE Input: %f, ", _G_leg[3](2));
		// printf("\n");
		// printf("\n");

		count = 0;
	}
	count++;
}

} // namespace legged_controllers
PLUGINLIB_EXPORT_CLASS(legged_controllers::LegController, controller_interface::ControllerBase)

