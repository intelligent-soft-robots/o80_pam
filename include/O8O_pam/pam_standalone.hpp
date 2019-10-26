#pragma once

#include "robot_interfaces/robot_backend.hpp"
#include "robot_interfaces/robot_frontend.hpp"
#include "robot_interfaces/robot_data.hpp"
#include "pam_interface/pressure_action.hpp"
#include "pam_interface/pam_robot_state.hpp"
#include "pam_interface/pam_actuator_state.hpp"
#include "pam_interface/driver.hpp"
#include "O8O/observation.hpp"
#include "O8O/back_end.hpp"
#include "O8O_pam/o8o_pam_actuator_state.hpp"

namespace O8O_pam
{



  template<int QUEUE_SIZE,int NB_ACTUATORS>
  class PamStandalone : public O8O::Standalone< QUEUE_SIZE,
						NB_ACTUATORS,
						pam_interface::PressureAction<NB_ACTUATORS>,
						// NB_ACTUATORS/2 : because 2 muscles per dof
						pam_interface::PamRobotState<NB_ACTUATORS/2>,
						O8O_pam::O8OPamActuatorState,
						pam_interface::PamRobotState<NB_ACTUATORS/2> >
  {

  public:

    PamStandalone( pam_interface::Driver<NB_ACTUATORS> &ri_driver,
		   double max_action_duration_s,
		   double max_inter_action_duration_s,
		   std::string segment_id,
		   std::string object_id );

    ~PamStandalone();
    
    O8O::States<NB_ACTUATORS,
		O8O_pam::O8OPamActuatorState>
    convert(const pam_interface::PamRobotState<NB_ACTUATORS/2> &robot_state);
    
    pam_interface::PressureAction<NB_ACTUATORS>
    convert(const O8O::States<NB_ACTUATORS,
	    O8O_pam::O8OPamActuatorState> &states);
    
  };

  #include "pam_standalone.hxx"
  
}
