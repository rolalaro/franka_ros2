#include <joint_controllers/interfaces.hpp>


namespace joint_controllers
{
const Interfaces Interfaces::position = Interfaces(0, "position");
const Interfaces Interfaces::velocity = Interfaces(1, "velocity");
const Interfaces Interfaces::effort = Interfaces(2, "effort");
}