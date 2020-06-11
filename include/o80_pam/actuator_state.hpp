#pragma once

#include "o80/state.hpp"
#include "o80/command_types.hpp"
#include "shared_memory/shared_memory.hpp"

namespace o80_pam
{
  class ActuatorState : public o80::State<int,ActuatorState>
{
public:
    ActuatorState();
    ActuatorState(int pressure);
};
}
