#pragma once

#include "pam_interface/pam_actuator_state.hpp"
#include "O8O/type.hpp"

namespace O8O_pam
{

  class O8OPamActuatorState : public pam_interface::PamActuatorState
  {

  public:

    O8OPamActuatorState()
      : pam_interface::PamActuatorState() {}

    O8OPamActuatorState(int pressure)
      : pam_interface::PamActuatorState(pressure) {}


    bool finished(const O8O::TimePoint &start,
		  const O8O::TimePoint &now,
		  const O8O::Duration_us& duration_us) const
    {
      throw std::runtime_error("to implement !");
      return true;
    }
	

    bool finished(const O8O::TimePoint &start,
		  const O8O::TimePoint &now,
		  const O8OPamActuatorState& current,
		  const O8O::Speed& speed) const
    {
      throw std::runtime_error("to implement !");
      return true;
    }

    
    O8OPamActuatorState intermediate_state(const O8O::TimePoint &start,
				     const O8O::TimePoint &now,
				     const O8OPamActuatorState &start_state,
				     const O8OPamActuatorState& current,
				     const O8O::Speed& speed) const
    {
      throw std::runtime_error("to implement !");
      return O8OPamActuatorState();
    }
    
    O8OPamActuatorState intermediate_state(const O8O::TimePoint &start,
				     const O8O::TimePoint &now,
				     const O8OPamActuatorState &start_state,
				     const O8OPamActuatorState &current,
				     const O8O::Duration_us& duration) const
    {
      throw std::runtime_error("to implement !");
      return O8OPamActuatorState();
    }

    
    O8OPamActuatorState intermediate_state(long int iteration_start,
					   long int iteration_now,
					   const O8OPamActuatorState &start_state,
					   const O8OPamActuatorState &current_state,
					   const O8OPamActuatorState &target_state,
					   const O8O::Iteration& iteration) const;


    template <class Archive>
    void serialize(Archive &archive){
      archive(pressure_);
    }

    friend shared_memory::private_serialization;

  };

  
}
