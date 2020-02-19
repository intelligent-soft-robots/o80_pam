#pragma once

#include "pam_interface/pam_actuator_state.hpp"
#include "O8O/type.hpp"

namespace O8O_pam
{

  class ActuatorState : public pam_interface::PamActuatorState
  {

  public:

    ActuatorState()
      : pam_interface::PamActuatorState() {}

    ActuatorState(int pressure)
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
		  const ActuatorState& start_state,
		  const ActuatorState& current_state,
		  const ActuatorState& end_state,
		  const O8O::Speed& speed) const;

    
    ActuatorState intermediate_state(const O8O::TimePoint &start,
					   const O8O::TimePoint &now,
					   const ActuatorState &start_state,
					   const ActuatorState& current,
					   const ActuatorState &end_state,
					   const O8O::Speed& speed) const;
    
    ActuatorState intermediate_state(const O8O::TimePoint &start,
					   const O8O::TimePoint &now,
					   const ActuatorState &start_state,
					   const ActuatorState &current,
					   const O8O::Duration_us& duration) const
    {
      throw std::runtime_error("to implement !");
      return ActuatorState();
    }

    
    ActuatorState intermediate_state(long int iteration_start,
					   long int iteration_now,
					   const ActuatorState &start_state,
					   const ActuatorState &current_state,
					   const ActuatorState &target_state,
					   const O8O::Iteration& iteration) const;


    template <class Archive>
    void serialize(Archive &archive){
      archive(pressure_);
    }

    friend shared_memory::private_serialization;

  };

  
}
