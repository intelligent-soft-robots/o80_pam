#include "O8O_pam/o8o_pam_actuator_state.hpp"

namespace O8O_pam
{


  O8OPamActuatorState
  O8OPamActuatorState::intermediate_state(long int iteration_start,
					  long int iteration_now,
					  const O8OPamActuatorState &start_state,
					  const O8OPamActuatorState &current_state,
					  const O8OPamActuatorState &target_state,
					  const O8O::Iteration& iteration) const
  {
    if (iteration_now>=iteration.value)
      {
	return O8OPamActuatorState(target_state.get_pressure());
      }
    int total_state = target_state.get_pressure()-start_state.get_pressure();
    int total_iteration = iteration.value - iteration_start;
    int diff_iteration = iteration_now - iteration_start;
    double ratio = static_cast<double>(diff_iteration) / static_cast<double>(total_iteration);
    double diff_state = ratio * static_cast<double>(total_state);
    double desired_state = diff_state + static_cast<double>(start_state.get_pressure());
    int desired = static_cast<int>(desired_state+0.5);
    return O8OPamActuatorState(desired);
  }


}
