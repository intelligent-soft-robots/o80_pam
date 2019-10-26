#pragma once
#include "shared_memory/serializer.hpp"
#include "O8O/state.hpp"

namespace O8O_pam:
{

  class PressureState : public O8O::State
  {

  public:

    PressureState();
    PressureState(int pressure);

    int get() const;
    
    PressureState intermediate_state(long int iteration_start,
				     long int iteration_now,
				     const PressureState &start_state,
				     const PressureState &current_state,
				     const PressureState &target_state,
				     const Iteration& iteration) const;

    template<class Archive>
    void serialize(Archive &archive){
      archive(pressure_);
    }
    
  private:
    
    friend shared_memory::private_serialization;
    int pressure_;
  }


}
