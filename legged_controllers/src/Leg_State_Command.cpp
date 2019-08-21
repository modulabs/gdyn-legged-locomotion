# include "Leg_State_Command.h"
class Leg_State_Command
{
    void Leg_State_Command::init()
    {
        q.resize(3);
        qdot.resize(3);
    };

    void Leg_State_Command::compute_error()
    {
        e.data = qd.data - q.data;
        e_dot.data = qd_dot.data - qdot.data;
        e_int.data = qd.data - q.data; //To do: re-definition
    }

}