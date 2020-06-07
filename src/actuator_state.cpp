
#include "o80_pam/actuator_state.hpp"

namespace o80_pam
{
ActuatorState::ActuatorState() : pressure_(-1)
{
}

ActuatorState::ActuatorState(int pressure) : pressure_(pressure)
{
}

int ActuatorState::get() const
{
    return pressure_;
}

void ActuatorState::set(int value)
{
    pressure_ = value;
}

std::string ActuatorState::to_string() const
{
    return std::to_string(pressure_);
}

bool ActuatorState::finished(const o80::TimePoint &start,
                             const o80::TimePoint &now,
                             const ActuatorState &start_state,
                             const ActuatorState &current,
                             const ActuatorState &previous_desired_state,
                             const ActuatorState &target_state,
                             const o80::Speed &speed) const
{
    return o80::finished(start,
                         now,
                         start_state.get(),
                         previous_desired_state.get(),
                         target_state.get(),
                         speed);
}

// speed: unit of pressure per second
ActuatorState ActuatorState::intermediate_state(
    const o80::TimePoint &start,
    const o80::TimePoint &now,
    const ActuatorState &start_state,
    const ActuatorState &current,
    const ActuatorState &previous_desired_state,
    const ActuatorState &target_state,
    const o80::Speed &speed) const
{
    return o80::intermediate_state(start,
                                   now,
                                   start_state.get(),
                                   previous_desired_state.get(),
                                   target_state.get(),
                                   speed);
}

ActuatorState ActuatorState::intermediate_state(
    long int start_iteration,
    long int current_iteration,
    const ActuatorState &start_state,
    const ActuatorState &current_state,
    const ActuatorState &previous_desired_state,
    const ActuatorState &target_state,
    const o80::Iteration &iteration) const
{
    return o80::intermediate_state(start_iteration,
                                   current_iteration,
                                   start_state.get(),
                                   previous_desired_state.get(),
                                   target_state.get(),
                                   iteration);
}

ActuatorState ActuatorState::intermediate_state(
    const o80::TimePoint &start,
    const o80::TimePoint &now,
    const ActuatorState &start_state,
    const ActuatorState &current_state,
    const ActuatorState &previous_desired_state,
    const ActuatorState &target_state,
    const o80::Duration_us &duration) const
{
    return o80::intermediate_state(start,
                                   now,
                                   start_state.get(),
                                   previous_desired_state.get(),
                                   target_state.get(),
                                   duration);
}
}
