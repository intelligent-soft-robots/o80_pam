
#include "O8O_pam/actuator_state.hpp"


namespace O8O_pam
{
  
  ActuatorState::ActuatorState()
    : pressure_(-1){}

  ActuatorState::ActuatorState(int pressure)
    : pressure_(pressure){}
  
  int ActuatorState::get() const
  {
    return pressure_;
  }
  
  void ActuatorState::set(int value)
  {
    pressure_=value;
  }

  std::string ActuatorState::to_string() const
  {
    return std::to_string(pressure_);
  }

  bool
  ActuatorState::finished(const O8O::TimePoint &start,
			  const O8O::TimePoint &now,
			  const ActuatorState& start_state,
			  const ActuatorState& current,
			  const ActuatorState& previous_desired_state,
			  const ActuatorState& target_state,
			  const O8O::Speed& speed) const
  {
    return O8O::finished(start,
			 now,
			 start_state.get(),
			 previous_desired_state.get(),
			 target_state.get(),
			 speed);
  }

  
  // speed: unit of pressure per second
  ActuatorState
  ActuatorState::intermediate_state(const O8O::TimePoint &start,
				    const O8O::TimePoint &now,
				    const ActuatorState &start_state,
				    const ActuatorState& current,
				    const ActuatorState& previous_desired_state,
				    const ActuatorState &target_state,
				    const O8O::Speed& speed) const
  {
    return O8O::intermediate_state(start,
				   now,
				   start_state.get(),
				   previous_desired_state.get(),
				   target_state.get(),
				   speed);
  }
  
  ActuatorState
  ActuatorState::intermediate_state(long int start_iteration,
				    long int current_iteration,
				    const ActuatorState &start_state,
				    const ActuatorState &current_state,
				    const ActuatorState &previous_desired_state,
				    const ActuatorState &target_state,
				    const O8O::Iteration &iteration) const
  {
    return O8O::intermediate_state(start_iteration,
				   current_iteration,
				   start_state.get(),
				   previous_desired_state.get(),
				   target_state.get(),
				   iteration);
  }

  ActuatorState
  ActuatorState::intermediate_state(const O8O::TimePoint &start,
				    const O8O::TimePoint &now,
				    const ActuatorState &start_state,
				    const ActuatorState &current_state,
				    const ActuatorState &previous_desired_state,
				    const ActuatorState &target_state,
				    const O8O::Duration_us& duration) const
  {
    return O8O::intermediate_state(start,
				   now,
				   start_state.get(),
				   previous_desired_state.get(),
				   target_state.get(),
				   duration);
  }
  
}
