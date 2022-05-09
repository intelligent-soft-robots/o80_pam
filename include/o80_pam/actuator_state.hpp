#pragma once

#include "o80/command_types.hpp"
#include "o80/state.hpp"
#include "shared_memory/shared_memory.hpp"

namespace o80_pam
{

/**
 * @brief ActuatorState class.
 * 
 * The ActuatorState class extends the O80 state class for
 * usage in PAM robotic actuators.
 */
class ActuatorState : public o80::State<int, ActuatorState>
{
public:
    /**
     * Constructor of ActuatorState object.
     */
    ActuatorState();

    /**
     * Constructor of Actuator State object with pressure
     * value predefined.
     * 
     * @param pressure pressure value.
     */
    ActuatorState(int pressure);
};
}  // namespace o80_pam
