#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl/chaindynparam.hpp>              // inverse dynamics



#include <boost/scoped_ptr.hpp>

#include "legged_controllers/ControllerJointState.h"

#include "Leg_State_Command.h"

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace legged_controllers{

	class LegController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		~LegController() 
		{
			_command_sub.shutdown();
		}

		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{	
			loop_count_ = 0;
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

			// urdf
			urdf::Model urdf;
			if (!urdf.initParam("robot_description"))
			{
				ROS_ERROR("Failed to parse urdf file");
            	return false;
			}

			// joint handle
			for (int i = 0; i < _n_joints; i++)
			{
				try
				{
					_joints.push_back(hw->getHandle(_joint_names[i]));
				}
				catch (const hardware_interface::HardwareInterfaceException &e)
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


			// 4. ********* KDL *********
			// 4.1 kdl parser
			if (!kdl_parser::treeFromUrdfModel(urdf, _kdl_tree))
			{
				ROS_ERROR("Failed to construct kdl tree");
				return false;
			}
			else
			{
				ROS_INFO("Constructed kdl tree");
			}

			// 4.2 kdl chain

			// 4.2.0 root and tip definition from hyq_control.yaml
			std::string root_name, tip_name_lf, tip_name_rf, tip_name_lh, tip_name_rh;
			
			if (!n.getParam("root_link", root_name))
			{
				ROS_ERROR("Could not find root link name");
				return false;
			}

			if (!n.getParam("tip_link/lf", tip_name_lf))
			{
				ROS_ERROR("Could not find tip link name: left front leg");
				return false;
			}

			if (!n.getParam("tip_link/rf", tip_name_rf))
			{
				ROS_ERROR("Could not find tip link name: right front leg");
				return false;
			}

			if (!n.getParam("tip_link/lh", tip_name_lh))
			{
				ROS_ERROR("Could not find tip link name: left hind leg");
				return false;
			}

			if (!n.getParam("tip_link/rh", tip_name_rh))
			{
				ROS_ERROR("Could not find tip link name: right hind leg");
				return false;
			}

			// 4.2.1 kdl chain: left front leg
			if (!_kdl_tree.getChain(root_name, tip_name_lf, _kdl_chain_lf))
			{
				ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
				ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name_lf);
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfJoints() << " joints");
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfSegments() << " segments");
				ROS_ERROR_STREAM("  The segments are:");

				KDL::SegmentMap segment_map = _kdl_tree.getSegments();
				KDL::SegmentMap::iterator it;

				for (it = segment_map.begin(); it != segment_map.end(); it++)
					ROS_ERROR_STREAM("    " << (*it).first);

				return false;
			}
			else
			{
				ROS_INFO("Got kdl chain of left front leg");
			}

			// 4.2.2 kdl chain: right front leg
			if (!_kdl_tree.getChain(root_name, tip_name_rf, _kdl_chain_rf))
			{
				ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
				ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name_rf);
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfJoints() << " joints");
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfSegments() << " segments");
				ROS_ERROR_STREAM("  The segments are:");

				KDL::SegmentMap segment_map = _kdl_tree.getSegments();
				KDL::SegmentMap::iterator it;

				for (it = segment_map.begin(); it != segment_map.end(); it++)
					ROS_ERROR_STREAM("    " << (*it).first);

				return false;
			}
			else
			{
				ROS_INFO("Got kdl chain of right front leg");
			}

			// 4.2.3 kdl chain: left hind leg
			if (!_kdl_tree.getChain(root_name, tip_name_lh, _kdl_chain_lh))
			{
				ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
				ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name_lh);
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfJoints() << " joints");
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfSegments() << " segments");
				ROS_ERROR_STREAM("  The segments are:");

				KDL::SegmentMap segment_map = _kdl_tree.getSegments();
				KDL::SegmentMap::iterator it;

				for (it = segment_map.begin(); it != segment_map.end(); it++)
					ROS_ERROR_STREAM("    " << (*it).first);

				return false;
			}
			else
			{
				ROS_INFO("Got kdl chain of left hind leg");
			}

			// 4.2.3 kdl chain: right hind leg
			if (!_kdl_tree.getChain(root_name, tip_name_rh, _kdl_chain_rh))
			{
				ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
				ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name_rh);
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfJoints() << " joints");
				ROS_ERROR_STREAM("  Tree has " << _kdl_tree.getNrOfSegments() << " segments");
				ROS_ERROR_STREAM("  The segments are:");

				KDL::SegmentMap segment_map = _kdl_tree.getSegments();
				KDL::SegmentMap::iterator it;

				for (it = segment_map.begin(); it != segment_map.end(); it++)
					ROS_ERROR_STREAM("    " << (*it).first);

				return false;
			}
			else
			{
				ROS_INFO("Got kdl chain of right hind leg");
			}

			// 4.3 inverse dynamics solver 초기화
			_gravity = KDL::Vector::Zero(); // @ To do: size????
			_gravity(2) = 9.81; // 0: x-axis 1: y-axis 2: z-axis

			_id_solver.reset(new KDL::ChainDynParam(_kdl_chain_lf, _gravity));

			// 4.5 forward kinematics solver 초기화
			_fk_pos_solver_lf.reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain_lf));
			_fk_pos_solver_rf.reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain_rf));
			_fk_pos_solver_lh.reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain_lh));
			_fk_pos_solver_rh.reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain_rh));

			// ********* 5. 각종 변수 초기화 *********

			// centroidal dynamics
			A_centroidal.resize(6, 12);
			b_centroidal.resize(6);

			// 5.1 KDL Vector & Matrix 초기화 (사이즈 정의 및 값 0)
			// _leg_lf.init();
			// _leg_rf.init();
			// _leg_lh.init();
			// _leg_rh.init();

			_leg_lf.qd.resize(_n_joints / 4);
			_leg_lf.qd_dot.resize(_n_joints / 4);
			_leg_lf.qd_ddot.resize(_n_joints / 4);
			_leg_lf.q.resize(_n_joints / 4);
			_leg_lf.qdot.resize(_n_joints / 4);
			_leg_lf.e.resize(_n_joints / 4);
			_leg_lf.e_dot.resize(_n_joints / 4);
			_leg_lf.e_int.resize(_n_joints / 4);

			_leg_rf.qd.resize(_n_joints / 4);
			_leg_rf.qd_dot.resize(_n_joints / 4);
			_leg_rf.qd_ddot.resize(_n_joints / 4);
			_leg_rf.q.resize(_n_joints / 4);
			_leg_rf.qdot.resize(_n_joints / 4);
			_leg_rf.e.resize(_n_joints / 4);
			_leg_rf.e_dot.resize(_n_joints / 4);
			_leg_rf.e_int.resize(_n_joints / 4);

			_leg_lh.qd.resize(_n_joints / 4);
			_leg_lh.qd_dot.resize(_n_joints / 4);
			_leg_lh.qd_ddot.resize(_n_joints / 4);
			_leg_lh.q.resize(_n_joints / 4);
			_leg_lh.qdot.resize(_n_joints / 4);
			_leg_lh.e.resize(_n_joints / 4);
			_leg_lh.e_dot.resize(_n_joints / 4);
			_leg_lh.e_int.resize(_n_joints / 4);

			_leg_rh.qd.resize(_n_joints / 4);
			_leg_rh.qd_dot.resize(_n_joints / 4);
			_leg_rh.qd_ddot.resize(_n_joints / 4);
			_leg_rh.q.resize(_n_joints / 4);
			_leg_rh.qdot.resize(_n_joints / 4);
			_leg_rh.e.resize(_n_joints / 4);
			_leg_rh.e_dot.resize(_n_joints / 4);
			_leg_rh.e_int.resize(_n_joints / 4);

			// command and state
			_tau_pid.resize(_n_joints / 4);
			_tau_damping.resize(_n_joints / 4);
			_tau_coulomb.resize(_n_joints / 4);
			_tau_fric.resize(_n_joints / 4);
			_tau_d_lf.resize(_n_joints / 4);

			_tau_d.resize(_n_joints);


			_qd.resize(_n_joints / 4);
			_qd_dot.resize(_n_joints / 4);
			_qd_ddot.resize(_n_joints / 4);

			_q.resize(_n_joints);
			_qdot.resize(_n_joints);

			// Dynamics
			_M.resize(_n_joints / 4);
			_C.resize(_n_joints / 4);
			_G.resize(_n_joints / 4);

			// gains
			_kp.resize(_n_joints);
			_kd.resize(_n_joints);

			std::vector<double> kp(_n_joints), kd(_n_joints);
			std::string gain_name;
        	for (size_t i = 0; i < _n_joints; i++)
        	{
				gain_name = "/hyq/leg_controller/gains/" + _joint_names[i] + "/p";
				if (n.getParam(gain_name, kp[i]))
				{
					_kp(i) = kp[i];
				}
				else
				{
					ROS_ERROR("Cannot find %s gain", gain_name.c_str());
					return false;
				}

				gain_name = "/hyq/leg_controller/gains/" + _joint_names[i] + "/d";
				if (n.getParam(gain_name, kp[i]))
				{
					_kp(i) = kp[i];
				}
				else
				{
					ROS_ERROR("Cannot find %s gain", gain_name.c_str());
					return false;
				}
        	}

			// command subscriber
			_commands_buffer.writeFromNonRT(std::vector<double>(_n_joints, 0.0));
			_command_sub = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &LegController::commandCB, this);

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

   			return true;
  		}

		void starting(const ros::Time& time)
		{
			_t = 0;

			// get joint positions
			for(size_t i=0; i<_n_joints; i++) 
			{
				_q(i) = _joints[i].getPosition();
			    _qdot(i) = _joints[i].getVelocity(); 
			}

			for(size_t i=0; i<_n_joints; i++) 
			{
				if(i < 3)
				{
					_leg_lf.q(i) = _joints[i].getPosition();
					_leg_lf.qdot(i) = _joints[i].getVelocity(); 
				}
				else if(i < 6)
				{
					_leg_rf.q(i-3) = _joints[i].getPosition();
					_leg_rf.qdot(i-3) = _joints[i].getVelocity(); 
				}
			    else if(i < 9)
				{
					_leg_lh.q(i-6) = _joints[i].getPosition();
					_leg_lh.qdot(i-6) = _joints[i].getVelocity(); 
				}
				else
				{
					_leg_rh.q(i-9) = _joints[i].getPosition();
					_leg_rh.qdot(i-9) = _joints[i].getVelocity(); 
				}
			}

			ROS_INFO("Starting Leg Controller");
		}

		void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
		{
			if(msg->data.size()!=_n_joints)
			{ 
			ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << _n_joints << ")! Not executing!");
			return; 
			}
			_commands_buffer.writeFromNonRT(msg->data);
		}

  		void update(const ros::Time& time, const ros::Duration& period)
  		{
			// ********* 0. Get states from gazebo *********
        	// 0.1 sampling time
			std::vector<double> & commands = *_commands_buffer.readFromRT(); // 이건 뭐지?
			double dt = period.toSec();

			_t += dt;

			// 0.2 get joint state
			for(size_t i=0; i<_n_joints; i++) 
			{
				_q(i) = _joints[i].getPosition();
			    _qdot(i) = _joints[i].getVelocity(); 
			}

			for(size_t i=0; i<_n_joints; i++) 
			{
				if(i < 3)
				{
					_leg_lf.q(i) = _joints[i].getPosition();
					_leg_lf.qdot(i) = _joints[i].getVelocity(); 
				}
				else if(i < 6)
				{
					_leg_rf.q(i-3) = _joints[i].getPosition();
					_leg_rf.qdot(i-3) = _joints[i].getVelocity(); 
				}
			    else if(i < 9)
				{
					_leg_lh.q(i-6) = _joints[i].getPosition();
					_leg_lh.qdot(i-6) = _joints[i].getVelocity(); 
				}
				else
				{
					_leg_rh.q(i-9) = _joints[i].getPosition();
					_leg_rh.qdot(i-9) = _joints[i].getVelocity(); 
				}
			}


			// 0.3 end-effector state by Compute forward kinematics (x, xdot)
			//     _x: position vector (3x1) + rotation matrix (3x3)
			_fk_pos_solver_lf->JntToCart(_leg_lf.q, _leg_lf.x);
			_fk_pos_solver_rf->JntToCart(_leg_rf.q, _leg_rf.x); 
			_fk_pos_solver_lh->JntToCart(_leg_lh.q, _leg_lh.x); 
			_fk_pos_solver_rh->JntToCart(_leg_rh.q, _leg_rh.x); 

			// ********* 1. Desired Trajecoty in Joint Space *********
			for (size_t i = 0; i < _n_joints/4; i++)
			{
				double f = 0.25;
				double A = 10.0;

				_leg_lf.qd_ddot(i) = -(2 * f * PI) * (2 * f * PI) * A * KDL::deg2rad * sin(2 * f * PI * _t);
				_leg_lf.qd_dot(i) = (2 * f * PI) * A * KDL::deg2rad * cos(2 * f * PI * _t);
				_leg_lf.qd(i) = A * KDL::deg2rad * sin(2 * f * PI * _t);
				
				_leg_rf.qd_ddot(i) = -(2 * f * PI) * (2 * f * PI) * A * KDL::deg2rad * sin(2 * f * PI * _t);
				_leg_rf.qd_dot(i) = (2 * f * PI) * A * KDL::deg2rad * cos(2 * f * PI * _t);
				_leg_rf.qd(i) = A * KDL::deg2rad * sin(2 * f * PI * _t);

				_leg_lh.qd_ddot(i) = -(2 * f * PI) * (2 * f * PI) * A * KDL::deg2rad * sin(2 * f * PI * _t);
				_leg_lh.qd_dot(i) = (2 * f * PI) * A * KDL::deg2rad * cos(2 * f * PI * _t);
				_leg_lh.qd(i) = A * KDL::deg2rad * sin(2 * f * PI * _t);

				_leg_rh.qd_ddot(i) = -(2 * f * PI) * (2 * f * PI) * A * KDL::deg2rad * sin(2 * f * PI * _t);
				_leg_rh.qd_dot(i) = (2 * f * PI) * A * KDL::deg2rad * cos(2 * f * PI * _t);
				_leg_rh.qd(i) = A * KDL::deg2rad * sin(2 * f * PI * _t);
			}

			// ********* 2. Motion Controller in Joint Space*********
			// *** 2.1 Error Definition in Joint Space ***
			//_leg_lf.compute_error();

			// // *** 2.2 Compute model(M,C,G) ***
			// _id_solver->JntToMass(_leg_lf.q, _M);
			// _id_solver->JntToCoriolis(_leg_lf.q, _leg_lf.qdot, _C);
			// _id_solver->JntToGravity(_leg_lf.q, _G);

			// // *** 2.3 Computed Friction ***
			// for (size_t i = 0; i < _n_joints/4; i++)
			// {
			// 	// // ver. 01 (friction o)
			// 	// _tau_damping(i) = 1.0 * qdot_(i);
			// 	// _tau_coulomb(i) = 1.0 * KDL::sign(qdot_(i));

			// 	// ver. 02 (friction x)
			// 	_tau_damping(i) = 0.0;
			// 	_tau_coulomb(i) = 0.0;
			// }

			// _tau_fric.data = _tau_damping.data + _tau_coulomb.data;

			// // // *** 2.4 Apply Torque Command to Actuator (lf leg) ***

			// // ver. 01 set torque by control law
			// for (size_t i = 0; i < _n_joints/4; i++)
			// {
			// 	_tau_pid(i) = _kp(i)*_leg_lf.e(i) + _kd(i)*_leg_lf.e_dot(i);
			// }

			// _tau_aux.data = _M.data * (_qd_ddot.data + _tau_pid.data);
			// _tau_comp.data = _C.data + _G.data + _tau_fric.data;
			// _tau_d_lf.data = _tau_aux.data + _tau_comp.data;

			// // // ver. 02 set torque zero
			// // for (size_t i = 0; i < _n_joints/4; i++)
			// // {
			// // 	_tau_d_lf(i) = 0.0;
			// // }



			// // for (int i = 0; i < _n_joints/4; i++)
			// // {
			// // 	// effort saturation
			// // 	if (_tau_d(i) >= _joint_urdfs[i]->limits->effort)
			// // 		_tau_d(i) = _joint_urdfs[i]->limits->effort;
				
			// // 	if (_tau_d(i) <= -_joint_urdfs[i]->limits->effort)
			// // 		_tau_d(i) = -_joint_urdfs[i]->limits->effort;

			// // 	// final torque input
			// // 	_joints[i].setCommand(_tau_d(i));
			// // }

			// // // *** 2.4 Apply Torque Command to Actuator (other legs) ***
			// // for (size_t i = 3; i < _n_joints; i++)
			// // {
			// // 	_tau_d(i) = 0.0;
			// // }

			// // // ***
			// // for (int i = 0; i < _n_joints; i++)
			// // {
			// // 	// effort saturation
			// // 	if (_tau_d(i) >= _joint_urdfs[i]->limits->effort)
			// // 		_tau_d(i) = _joint_urdfs[i]->limits->effort;
				
			// // 	if (_tau_d(i) <= -_joint_urdfs[i]->limits->effort)
			// // 		_tau_d(i) = -_joint_urdfs[i]->limits->effort;

			// // 	// final torque input
			// // 	_joints[i].setCommand(_tau_d(i));
			// // }

			// *** 2.4 Apply Torque Command to Actuator ***
			for (int i = 0; i < _n_joints; i++)
			{
				if(i < 3)
				{
					_tau_d(i) = _tau_d_lf(i);
				}
				else if(i < 6)
				{
					_tau_d(i) = 0.0;
				}
				else if(i < 9)
				{
					_tau_d(i) = 0.0;
				}
				else
				{
					_tau_d(i) = 0.0;
				}
			}
				
			for (int i = 0; i < _n_joints; i++)
			{
				// effort saturation
				if (_tau_d(i) >= _joint_urdfs[i]->limits->effort)
					_tau_d(i) = _joint_urdfs[i]->limits->effort;
				
				if (_tau_d(i) <= -_joint_urdfs[i]->limits->effort)
					_tau_d(i) = -_joint_urdfs[i]->limits->effort;

				// final torque input
				_joints[i].setCommand(_tau_d(i));
			}





			// ********* 4. state 출력 *********
			print_state();
			
			// publish
			if (loop_count_ % 10 == 0)
			{
				if (_controller_state_pub->trylock())
				{
					_controller_state_pub->msg_.header.stamp = time;
					for(int i=0; i<_n_joints; i++)
					{
						// _controller_state_pub->msg_.command[i] = R2D*_qd(i);
						// _controller_state_pub->msg_.command_dot[i] = R2D*_qd_dot(i);
						// _controller_state_pub->msg_.state[i] = R2D*_q(i);
						// _controller_state_pub->msg_.state_dot[i] = R2D*_qdot(i);
						// _controller_state_pub->msg_.q_error[i] = R2D*_e(i);
						// _controller_state_pub->msg_.qdot_error[i] = R2D*_e_dot(i);
						// _controller_state_pub->msg_.effort_command[i] = _tau_d(i);
						// _controller_state_pub->msg_.effort_feedback[i] = _tau_d(i) - _controller_state_pub->msg_.effort_feedforward[i];
					}
					_controller_state_pub->unlockAndPublish();
				}
			}
  		}

  		void stopping(const ros::Time& time) { }

		void enforceJointLimits(double &command, unsigned int index)
		{
			// Check that this joint has applicable limits
			if (_joint_urdfs[index]->type == urdf::Joint::REVOLUTE || _joint_urdfs[index]->type == urdf::Joint::PRISMATIC)
			{
			if( command > _joint_urdfs[index]->limits->upper ) // above upper limnit
			{
				command = _joint_urdfs[index]->limits->upper;
			}
			else if (command < _joint_urdfs[index]->limits->lower) // below lower limit
			{
				command = _joint_urdfs[index]->limits->lower;
			}
			}
		}

		void print_state()
		{
			static int count = 0;
			if (count > 990)
			{
				printf("*********************************************************\n\n");
				printf("*** Simulation Time (unit: sec)  ***\n");
				printf("t = %f\n", _t);
				printf("\n");

				printf("*********************** Left Front Leg **********************************\n\n");
				printf("*** Desired Position in Joint Space (unit: deg) ***\n");
				printf("qd(1): %f, ", _leg_lf.qd(0) * R2D);
				printf("qd(2): %f, ", _leg_lf.qd(1) * R2D);
				printf("qd(3): %f, ", _leg_lf.qd(2) * R2D);
				printf("\n");
				printf("\n");

				printf("*** Actual Position in Joint Space (unit: deg) ***\n");
				// printf("q(1): %f, ", _q(0) * R2D);
				// printf("q(2): %f, ", _q(1) * R2D);
				// printf("q(3): %f, ", _q(2) * R2D);
				// printf("\n");
				printf("q(1): %f, ", _leg_lf.q(0) * R2D);
				printf("q(2): %f, ", _leg_lf.q(1) * R2D);
				printf("q(3): %f, ", _leg_lf.q(2) * R2D);
				printf("\n");
				printf("\n");
				
				// printf("*** Error Position in Joint Space (unit: deg) ***\n");
				// printf("e(1): %f, ", _e_lf(0) * R2D);
				// printf("e(2): %f, ", _e_lf(1) * R2D);
				// printf("e(3): %f, ", _e_lf(2) * R2D);
				// printf("\n");
				// printf("\n");

				printf("*** Actual Position in Task Space (unit: m) ***\n");
				printf("x(1): %f, ", _leg_lf.x.p(0) * 1);
				printf("x(2): %f, ", _leg_lf.x.p(1) * 1);
				printf("x(3): %f, ", _leg_lf.x.p(2) * 1);
				printf("\n");
				printf("\n");
				
				// printf("*** Torque Input (unit: Nm) ***\n");
				// printf("Hip AA Input: %f, ", _tau_d(0));
				// printf("Hip FE Input: %f, ", _tau_d(1));
				// printf("Knee FE Input: %f, ", _tau_d(2));
				// printf("\n");
				// printf("\n");

				printf("*********************** Right Front Leg **********************************\n\n");
				printf("*** Actual Position in Joint Space (unit: deg) ***\n");
				// printf("q(1): %f, ", _q(3) * R2D);
				// printf("q(2): %f, ", _q(4) * R2D);
				// printf("q(3): %f, ", _q(5) * R2D);
				// printf("\n");
				printf("q(1): %f, ", _leg_rf.q(0) * R2D);
				printf("q(2): %f, ", _leg_rf.q(1) * R2D);
				printf("q(3): %f, ", _leg_rf.q(2) * R2D);
				printf("\n");
				printf("\n");
				

				printf("*** Actual Position in Task Space (unit: m) ***\n");
				printf("x(1): %f, ", _leg_rf.x.p(0) * 1);
				printf("x(2): %f, ", _leg_rf.x.p(1) * 1);
				printf("x(3): %f, ", _leg_rf.x.p(2) * 1);
				printf("\n");
				printf("\n");

				// printf("*** Torque Input (unit: Nm) ***\n");
				// printf("Hip AA Input: %f, ", _tau_d(3));
				// printf("Hip FE Input: %f, ", _tau_d(4));
				// printf("Knee FE Input: %f, ", _tau_d(5));
				// printf("\n");
				// printf("\n");

				printf("*********************** Left Hind Leg **********************************\n\n");
				printf("*** Actual Position in Joint Space (unit: deg) ***\n");
				// printf("q(1): %f, ", _q(6) * R2D);
				// printf("q(2): %f, ", _q(7) * R2D);
				// printf("q(3): %f, ", _q(8) * R2D);
				// printf("\n");
				printf("q(1): %f, ", _leg_lh.q(0) * R2D);
				printf("q(2): %f, ", _leg_lh.q(1) * R2D);
				printf("q(3): %f, ", _leg_lh.q(2) * R2D);
				printf("\n");
				printf("\n");

				printf("*** Actual Position in Task Space (unit: m) ***\n");
				printf("x(1): %f, ", _leg_lh.x.p(0) * 1);
				printf("x(2): %f, ", _leg_lh.x.p(1) * 1);
				printf("x(3): %f, ", _leg_lh.x.p(2) * 1);
				printf("\n");
				printf("\n");

				// printf("*** Torque Input (unit: Nm) ***\n");
				// printf("Hip AA Input: %f, ", _tau_d(6));
				// printf("Hip FE Input: %f, ", _tau_d(7));
				// printf("Knee FE Input: %f, ", _tau_d(8));
				// printf("\n");
				// printf("\n");

				printf("*********************** Right Hind Leg **********************************\n\n");
				printf("*** Actual Position in Joint Space (unit: deg) ***\n");
				// printf("q(1): %f, ", _q(9) * R2D);
				// printf("q(2): %f, ", _q(10) * R2D);
				// printf("q(3): %f, ", _q(11) * R2D);
				// printf("\n");
				printf("q(1): %f, ", _leg_rh.q(0) * R2D);
				printf("q(2): %f, ", _leg_rh.q(1) * R2D);
				printf("q(3): %f, ", _leg_rh.q(2) * R2D);
				printf("\n");
				printf("\n");

				printf("*** Actual Position in Task Space (unit: m) ***\n");
				printf("x(1): %f, ", _leg_rh.x.p(0) * 1);
				printf("x(2): %f, ", _leg_rh.x.p(1) * 1);
				printf("x(3): %f, ", _leg_rh.x.p(2) * 1);
				printf("\n");
				printf("\n");

				// printf("*** Torque Input (unit: Nm) ***\n");
				// printf("Hip AA Input: %f, ", _tau_d(9));
				// printf("Hip FE Input: %f, ", _tau_d(10));
				// printf("Knee FE Input: %f, ", _tau_d(11));
				// printf("\n");
				// printf("\n");




				





				// printf("*** Actual Position in Joint Space (unit: deg) ***\n");
				// printf("q(1): %f, ", _q(0) * R2D);
				// printf("q(2): %f, ", _q(1) * R2D);
				// printf("q(3): %f, ", _q(2) * R2D);
				// printf("\n");
				// printf("\n");

				// printf("*** Error Position in Joint Space (unit: deg) ***\n");
				// printf("e(1): %f, ", _e(0) * R2D);
				// printf("e(2): %f, ", _e(1) * R2D);
				// printf("e(3): %f, ", _e(2) * R2D);
				// printf("\n");
				// printf("\n");

				// printf("*** Desired Position in Task Space (unit: m) ***\n");
				// printf("xd(1): %f, ", _xd.p(0) * 1);
				// printf("xd(2): %f, ", _xd.p(1) * 1);
				// printf("xd(3): %f, ", _xd.p(2) * 1);
				// printf("\n");
				// printf("\n");

				// printf("*** Actual Position in Task Space (unit: m) ***\n");
				// printf("x(1): %f, ", _x.p(0) * 1);
				// printf("x(2): %f, ", _x.p(1) * 1);
				// printf("x(3): %f, ", _x.p(2) * 1);
				// printf("\n");
				// printf("\n");

				// printf("*** Torque Input (unit: Nm) ***\n");
				// printf("Input(1): %f, ", _tau_d(0));
				// printf("Input(2): %f, ", _tau_d(1));
				// printf("Input(3): %f, ", _tau_d(2));
				// printf("\n");
				// printf("\n");

				// printf("*** Mass Matrix ***\n");
				// printf("%f, ", _M(0, 0));
				// printf("%f, ", _M(0, 1));
				// printf("%f, ", _M(0, 2));
				// printf("\n");

				// printf("%f, ", _M(1, 0));
				// printf("%f, ", _M(1, 1));
				// printf("%f, ", _M(1, 2));
				// printf("\n");

				// printf("%f, ", _M(2, 0));
				// printf("%f, ", _M(2, 1));
				// printf("%f ,", _M(2, 2));
				// printf("\n");
				// printf("\n");

				count = 0;
			}
			count++;
		}

	  private:
		int loop_count_;
		double _t;

		// joint handles
		unsigned int _n_joints;
		std::vector<std::string> _joint_names;
  		std::vector<hardware_interface::JointHandle> _joints;
		std::vector<urdf::JointConstSharedPtr> _joint_urdfs;

		// kdl
		KDL::Tree 	_kdl_tree;
		KDL::Chain	_kdl_chain_lf;
		KDL::Chain	_kdl_chain_rf;
		KDL::Chain	_kdl_chain_lh;
		KDL::Chain	_kdl_chain_rh;

		// kdl M,C,G
		KDL::JntSpaceInertiaMatrix _M; // intertia matrix
		KDL::JntArray _C;              // coriolis
		KDL::JntArray _G;              // gravity torque vector
		KDL::Vector _gravity;

		// kdl solver
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> _fk_pos_solver_lf; //Solver to compute the forward kinematics (position): left front leg
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> _fk_pos_solver_rf; //Solver to compute the forward kinematics (position): right front leg
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> _fk_pos_solver_lh; //Solver to compute the forward kinematics (position): left hind leg
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> _fk_pos_solver_rh; //Solver to compute the forward kinematics (position): right hind leg
		boost::scoped_ptr<KDL::ChainDynParam> _id_solver;               // Solver To compute the inverse dynamics

		// Friction Model
		KDL::JntArray _tau_damping, _tau_coulomb, _tau_fric;



		// centroidal dynamic model
		Eigen::MatrixXd A_centroidal;
		Eigen::VectorXd b_centroidal;

		double m_centroidal;
		Eigen::MatrixXd inertia_centroidal;
		
		// task space state
		Eigen::VectorXd P_leg_lf;
		Eigen::VectorXd P_leg_rf;
		Eigen::VectorXd P_leg_lh;
		Eigen::VectorXd P_leg_rh;

		Eigen::VectorXd P_body_com;
		Eigen::VectorXd P_dot_body_com;
		Eigen::VectorXd P_ddot_body_com;

		Eigen::VectorXd w_body_com;
		Eigen::VectorXd w_dot_body_com;

		// Task Space State  (x.p: frame position(3x1), x.m: frame orientation (3x3))
		KDL::Frame _xd; // To do

		// Joint Space State
		KDL::JntArray _q, _qdot;
		KDL::JntArray _qd, _qd_dot, _qd_ddot; // To do

		// Input
		KDL::JntArray _tau_aux;
		KDL::JntArray _tau_comp;
		KDL::JntArray _tau_pid;
		KDL::JntArray _tau_d_lf;
		KDL::JntArray _tau_d;

		//
		realtime_tools::RealtimeBuffer<std::vector<double> > _commands_buffer;

		// gain
		KDL::JntArray _kp, _kd;

		// topic
		ros::Subscriber _command_sub;
		boost::scoped_ptr<
			realtime_tools::RealtimePublisher<
				legged_controllers::ControllerJointState> > _controller_state_pub;


		//
		Leg_State_Command _leg_lf, _leg_rf, _leg_lh, _leg_rh;
	};

}

PLUGINLIB_EXPORT_CLASS(legged_controllers::LegController, controller_interface::ControllerBase)

