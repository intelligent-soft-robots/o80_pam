#include "O8O_pam/actuator_state.hpp"

namespace O8O_pam
{


  bool
  ActuatorState::finished(const O8O::TimePoint &start,
				const O8O::TimePoint &now,
				const ActuatorState& start_state,
				const ActuatorState& current,
				const ActuatorState& target_state,
				const O8O::Speed& speed) const
  {
    long int start_pressure = start_state.get_pressure();
    long int end_pressure = target_state.get_pressure();
    long int pressure_diff = abs(end_pressure - start_pressure);
    double duration_d = static_cast<double>(pressure_diff) / static_cast<double>(speed.value);
    long int duration = static_cast<long int>(duration_d*1E6);
    long int expected_end_int = start.count()+duration;
    O8O::TimePoint expected_end(expected_end_int);
    if(now>expected_end)
      {
	return true;
      }
    return false;
  }

  
  // speed: unit of pressure per second
  ActuatorState
  ActuatorState::intermediate_state(const O8O::TimePoint &start,
					  const O8O::TimePoint &now,
					  const ActuatorState &start_state,
					  const ActuatorState& current,
					  const ActuatorState &end_state,
					  const O8O::Speed& speed) const
  {
    double time_diff = static_cast<double>((now-start).count())/1E6; // seconds
    long int start_pressure = start_state.get_pressure();
    long int end_pressure = end_state.get_pressure();
    long int pressure_diff = end_pressure - start_pressure;
    double desired;
    if(pressure_diff>0)
      {
	desired = start_pressure+static_cast<double>(speed.value) * time_diff;
	if(desired>end_pressure)
	  {
	    return end_pressure;
	  }
	return ActuatorState(static_cast<int>(desired+0.5));
      }
    desired = start_pressure-static_cast<double>(speed.value) * time_diff;
    if(desired<end_pressure)
      {
	return end_pressure;
      }
    return ActuatorState(static_cast<int>(desired+0.5));
  }
  


  ActuatorState
  ActuatorState::intermediate_state(long int iteration_start,
					  long int iteration_now,
					  const ActuatorState &start_state,
					  const ActuatorState &current_state,
					  const ActuatorState &target_state,
					  const O8O::Iteration& iteration) const
  {
    if (iteration_now>=iteration.value)
      {
	return ActuatorState(target_state.get_pressure());
      }
    int total_state = target_state.get_pressure()-start_state.get_pressure();
    int total_iteration = iteration.value - iteration_start;
    int diff_iteration = iteration_now - iteration_start;
    double ratio = static_cast<double>(diff_iteration) / static_cast<double>(total_iteration);
    double diff_state = ratio * static_cast<double>(total_state);
    double desired_state = diff_state + static_cast<double>(start_state.get_pressure());
    int desired = static_cast<int>(desired_state+0.5);
    return ActuatorState(desired);
  }


}
