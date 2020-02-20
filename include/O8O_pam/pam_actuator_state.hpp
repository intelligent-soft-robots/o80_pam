#pragma once
#include "shared_memory/serializer.hpp"

namespace pam_interface
{

  class PamActuatorState 
  {

  public:

    PamActuatorState();
    PamActuatorState(int pressure);
    ~PamActuatorState(){}
    
    int get_pressure() const;
    void set_pressure(int value);
    

  protected:
    
    int pressure_;

  };
  

}
