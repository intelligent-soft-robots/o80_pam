#pragma once

#include "shared_memory/shared_memory.hpp"

#include "O8O/interpolation.hpp"
#include "O8O/type.hpp"

namespace O8O_pam
{

  class ActuatorState 
  {

  public:

    ActuatorState();

    ActuatorState(int pressure);

    int get() const;
    
    void set(int value);

    std::string to_string() const;
    
    bool finished(const O8O::TimePoint &start,
		  const O8O::TimePoint &now,
		  const O8O::Duration_us& duration_us) const;

    bool finished(const O8O::TimePoint &start,
		  const O8O::TimePoint &now,
		  const ActuatorState& start_state,
		  const ActuatorState& current_state,
		  const ActuatorState& previous_desired_state,
		  const ActuatorState& target_state,
		  const O8O::Speed& speed) const;
    
    ActuatorState intermediate_state(const O8O::TimePoint &start,
				     const O8O::TimePoint &now,
				     const ActuatorState &start_state,
				     const ActuatorState& current_state,
				     const ActuatorState& previous_desired_state,
				     const ActuatorState &target_state,
				     const O8O::Speed& speed) const;
    
    ActuatorState intermediate_state(const O8O::TimePoint &start,
				     const O8O::TimePoint &now,
				     const ActuatorState &start_state,
				     const ActuatorState &current_state,
				     const ActuatorState& previous_desired_state,
				     const ActuatorState& target_state,
				     const O8O::Duration_us& duration) const;
    
    ActuatorState intermediate_state(long int start_iteration,
				     long int current_iteration,
				     const ActuatorState &start_state,
				     const ActuatorState &current_state,
				     const ActuatorState& previous_desired_state,
				     const ActuatorState &target_state,
				     const O8O::Iteration& iteration) const;

    template <class Archive>
    void serialize(Archive &archive){
      archive(pressure_);
    }

  private:

    friend shared_memory::private_serialization;
    
    int pressure_;
    
  };
  
}
