#include <kdl/kdl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

class Leg_State_Command
{
    public:
        KDL::JntArray qd, qd_dot, qd_ddot;
        KDL::JntArray q, qdot;
        KDL::JntArray e, e_dot, e_int;

        //KDL::Frame xd, xd_dot, xd_dotdot;
        KDL::Frame x, xdot;
        //KDL::JntArray ex, ex_dot, ex_int; 

    public:
        void init();
        void compute_error();

};