#pragma once

#include "shared_memory/shared_memory.hpp"

#include "o80/interpolation.hpp"
#include "o80/type.hpp"

namespace o80_pam
{

  class ActuatorState 
  {

  public:

    ActuatorState();

    ActuatorState(int pressure);

    int get() const;
    
    void set(int value);

    std::string to_string() const;
    
    bool finished(const o80::TimePoint &start,
		  const o80::TimePoint &now,
		  const o80::Duration_us& duration_us) const;

    bool finished(const o80::TimePoint &start,
		  const o80::TimePoint &now,
		  const ActuatorState& start_state,
		  const ActuatorState& current_state,
		  const ActuatorState& previous_desired_state,
		  const ActuatorState& target_state,
		  const o80::Speed& speed) const;
    
    ActuatorState intermediate_state(const o80::TimePoint &start,
				     const o80::TimePoint &now,
				     const ActuatorState &start_state,
				     const ActuatorState& current_state,
				     const ActuatorState& previous_desired_state,
				     const ActuatorState &target_state,
				     const o80::Speed& speed) const;
    
    ActuatorState intermediate_state(const o80::TimePoint &start,
				     const o80::TimePoint &now,
				     const ActuatorState &start_state,
				     const ActuatorState &current_state,
				     const ActuatorState& previous_desired_state,
				     const ActuatorState& target_state,
				     const o80::Duration_us& duration) const;
    
    ActuatorState intermediate_state(long int start_iteration,
				     long int current_iteration,
				     const ActuatorState &start_state,
				     const ActuatorState &current_state,
				     const ActuatorState& previous_desired_state,
				     const ActuatorState &target_state,
				     const o80::Iteration& iteration) const;

    template <class Archive>
    void serialize(Archive &archive){
      archive(pressure_);
    }

  private:

    friend shared_memory::private_serialization;
    
    int pressure_;
    
  };
  
}
