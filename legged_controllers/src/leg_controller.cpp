#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>

#include "legged_controllers/ControllerJointState.h"

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
			// get joint positions
			for(size_t i=0; i<_n_joints; i++) 
			{
				_q(i) = _joints[i].getPosition();
				_qdot(i) = _joints[i].getVelocity();
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
			std::vector<double> & commands = *_commands_buffer.readFromRT();
			double dt = period.toSec();
			double q_d_old;

			// get joint states
			static double t = 0;
			for (size_t i=0; i<_n_joints; i++)
			{
				_q_d(i) = 0*D2R*sin(PI/2*t);
				// _q_d(i) = 30*D2R*sin(PI/2*t);
				//_q_d(i) = commands[i];

				enforceJointLimits(_q_d(i), i);
				_q(i) = _joints[i].getPosition();
				_qdot(i) = _joints[i].getVelocity();

		        // Compute position error
				angles::shortest_angular_distance_with_limits(
					_q(i),
					_q_d(i),
					_joint_urdfs[i]->limits->lower,
					_joint_urdfs[i]->limits->upper,
					_q_error(i));

				_qdot_d(i) = 0*D2R*PI/2*cos(PI/2*t); // (_q_d(i) - _q_d_old(i)) / period.toSec();;
				_qddot_d(i) = -0*D2R*PI*PI/2/2*sin(PI/2*t); // (_qdot_d(i) - _qdot_d_old(i)) / period.toSec();

				_qdot_error(i) = _qdot_d(i) - _qdot(i);

				_q_d_old(i) = _q_d(i);
				_qdot_d_old(i) = _qdot_d(i);

				// friction compensation, to do: implement friction observer
				_tau_fric(i) = 1*_qdot(i) + 1*KDL::sign(_qdot(i));
			}
			_q_d(0) = 0;
			_q_d(1) = -45;
			_q_d(2) = 45;
			_q_d(3) = 0;
			_q_d(4) = -45;
			_q_d(5) = 45;
			_q_d(6) = 0;
			_q_d(7) = -45;
			_q_d(8) = 45;
			_q_d(9) = 0;
			_q_d(10) = -45;
			_q_d(11) = 45;

			
			t += dt;
			
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
			if (loop_count_ % 10 == 0)
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
			else if( command < _joint_urdfs[index]->limits->lower ) // below lower limit
			{
				command = _joint_urdfs[index]->limits->lower;
			}
			}
		}
	private:
		int loop_count_;

		// joint handles
		unsigned int _n_joints;
		std::vector<std::string> _joint_names;
  		std::vector<hardware_interface::JointHandle> _joints;
		std::vector<urdf::JointConstSharedPtr> _joint_urdfs;

		// kdl
		KDL::Tree 	_kdl_tree;
		KDL::Chain	_kdl_chain;


		// cmd, state
		realtime_tools::RealtimeBuffer<std::vector<double> > _commands_buffer;
		KDL::JntArray _tau_d, _tau_fric;
		KDL::JntArray _q_d, _qdot_d, _qddot_d, _q_d_old, _qdot_d_old;
		KDL::JntArray _q, _qdot;
		KDL::JntArray _q_error, _qdot_error;

		// gain
		KDL::JntArray _kp, _kd;

		// topic
		ros::Subscriber _command_sub;
		boost::scoped_ptr<
			realtime_tools::RealtimePublisher<
				legged_controllers::ControllerJointState> > _controller_state_pub;
	};

}

PLUGINLIB_EXPORT_CLASS(legged_controllers::LegController, controller_interface::ControllerBase)

