
template<int QUEUE_SIZE,int NB_ACTUATORS>
PamStandalone<QUEUE_SIZE,NB_ACTUATORS>::PamStandalone(pam_interface::Driver<NB_ACTUATORS>
						      &ri_driver,
						      double max_action_duration_s,
						      double max_inter_action_duration_s,
						      std::string segment_id,
						      std::string object_id )
  : O8O::Standalone<QUEUE_SIZE,
		    NB_ACTUATORS,
		    pam_interface::PressureAction<NB_ACTUATORS>,
		    pam_interface::PamRobotState<NB_ACTUATORS/2>,
		    O8O_pam::O8OPamActuatorState,
		    pam_interface::PamRobotState<NB_ACTUATORS/2> >
  (ri_driver,
   max_action_duration_s,
   max_inter_action_duration_s,
   segment_id,
   object_id)
{}


template<int QUEUE_SIZE,int NB_ACTUATORS>
PamStandalone<QUEUE_SIZE,NB_ACTUATORS>::~PamStandalone()
{}


template<int QUEUE_SIZE,int NB_ACTUATORS>
O8O::States<NB_ACTUATORS,
	    O8O_pam::O8OPamActuatorState>
PamStandalone<QUEUE_SIZE,NB_ACTUATORS>::convert(const pam_interface::PamRobotState<NB_ACTUATORS/2> &robot_state)
{
  
  O8O::States<NB_ACTUATORS,O8O_pam::O8OPamActuatorState> states;

  int dof;
  for(unsigned int actuator=0;actuator<NB_ACTUATORS;actuator++)
    {
      dof = actuator/2;
      if(actuator%2==0)
	{
	  states.values[actuator].set_pressure(robot_state.get_joint_state(dof).agonist);
	}
      else
	{
	  states.values[actuator].set_pressure(robot_state.get_joint_state(dof).antagonist);
	}
    }

  return states;

}

template<int QUEUE_SIZE,int NB_ACTUATORS>
pam_interface::PressureAction<NB_ACTUATORS>
PamStandalone<QUEUE_SIZE,NB_ACTUATORS>::convert(const O8O::States<NB_ACTUATORS,
						O8O_pam::O8OPamActuatorState> &states)
{

  pam_interface::PressureAction<NB_ACTUATORS> action;

  int dof;
  
  for(unsigned int actuator=0;actuator<NB_ACTUATORS;actuator++)
    {
      action.set(actuator,
		 states.values[actuator].get_pressure());
    }

  return action;

}
