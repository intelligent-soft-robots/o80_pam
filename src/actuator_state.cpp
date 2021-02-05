
#include "o80_pam/actuator_state.hpp"

namespace o80_pam
{
ActuatorState::ActuatorState() : o80::State<int, ActuatorState>(0)
{
}

ActuatorState::ActuatorState(int pressure)
    : o80::State<int, ActuatorState>(pressure)
{
}

}  // namespace o80_pam
