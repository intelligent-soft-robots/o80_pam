#pragma once

#include "robot_interfaces/robot_backend.hpp"
#include "robot_interfaces/robot_frontend.hpp"
#include "robot_interfaces/robot_data.hpp"

#include "pam_interface/pressure_action.hpp"
#include "pam_interface/state/robot.hpp"
#include "pam_interface/state/joint.hpp"
#include "pam_interface/driver.hpp"

#include "O8O/observation.hpp"
#include "O8O/back_end.hpp"

#include "O8O_pam/actuator_state.hpp"

namespace O8O_pam
{

  template<int QUEUE_SIZE,int NB_ACTUATORS>
  class Standalone : public O8O::Standalone< QUEUE_SIZE,
						NB_ACTUATORS,
						pam_interface::PressureAction<NB_ACTUATORS>,
						// NB_ACTUATORS/2 : because 2 muscles per dof
						pam_interface::RobotState<NB_ACTUATORS/2>,
						O8O_pam::ActuatorState,
						pam_interface::RobotState<NB_ACTUATORS/2> >
  {

  public:

    typename std::shared_ptr<pam_interface::Driver<NB_ACTUATORS>> DriverPtr;
    
    Standalone( DriverPtr &ri_driver,
		double frequency,
		std::string segment_id,
		std::string object_id);

    ~Standalone();
    
    O8O::States<NB_ACTUATORS,
		O8O_pam::ActuatorState>
    convert(const pam_interface::RobotState<NB_ACTUATORS/2> &robot_state);
    
    pam_interface::PressureAction<NB_ACTUATORS>
    convert(const O8O::States<NB_ACTUATORS,
	    O8O_pam::ActuatorState> &states);

    void
    enrich_extended_state(pam_interface::RobotState<NB_ACTUATORS/2> &extended_state,
			  const pam_interface::RobotState<NB_ACTUATORS/2> &ri_observation);

    
  };

  #include "pam_standalone.hxx"
  
}
