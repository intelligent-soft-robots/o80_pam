#include "O8O_pam/pressure_state.hpp"

namespace O8O_pam
{

  PressureState::PressureState()
    :pressure_(-1){}

  PressureState::PressureState(int pressure)
    :pressure_(pressure){}

  std::string PressureState::to_string()
  {
    return std::to_string(pressure_);
  }

  PressureState PressureState::eval(int pressure)
  {
    return PressureState(pressure);
  }

  void PressureState::set(int pressure)
  {
    pressure_ = pressure;
  }

  int PressureState::get() const
  {
    return pressure_;
  }

  bool PressureState::finished (const TimePoint& start_time,
				const TimePoint& now,
				const PressureState &start_state,
				const PressureState &current_state,
				const PressureState &target_state,
				double speed) const
  {
    return O8O::finished(start_time,now,
			 start_state.get(),
			 current_state.get(),
			 target_state.get(),
			 speed);
  }
    
  PressureState PressureState::intermediate_state(long int iteration_start,
						  long int iteration_now,
						  const PressureState &start_state,
						  const PressureState &current_state,
						  const PressureState &target_state,
						  const Iteration& iteration) const
  {
    int pressure = O8O::intermediate_state(iteration_start,
					   iteration_now,
					   start_state.get(),
					   current_state.get(),
					   target_state.get(),
					   iteration.value);
    return PressureState(pressure);
  }

  
}
