
template<int QUEUE_SIZE,int NB_ACTUATORS,class DRIVER>
Standalone<QUEUE_SIZE,NB_ACTUATORS,DRIVER>::Standalone(DriverPtr &ri_driver,
						       double max_action_duration_s,
						       double max_inter_action_duration_s,
						       double frequency,
						       std::string segment_id)
  : O8O::Standalone<QUEUE_SIZE,
		    NB_ACTUATORS,
		    pam_interface::PressureAction<NB_ACTUATORS>,
		    pam_interface::RobotState<NB_ACTUATORS/2>,
		    O8O_pam::ActuatorState,
		    O8O::EmptyExtendedState>
  (ri_driver,
   max_action_duration_s,
   max_inter_action_duration_s,
   frequency,
   segment_id)
{
}


template<int QUEUE_SIZE,int NB_ACTUATORS,class DRIVER>
Standalone<QUEUE_SIZE,NB_ACTUATORS,DRIVER>::~Standalone()
{}


template<int QUEUE_SIZE,int NB_ACTUATORS,class DRIVER>
O8O::States<NB_ACTUATORS,
	    O8O_pam::ActuatorState>
Standalone<QUEUE_SIZE,
	   NB_ACTUATORS,DRIVER>::convert(const pam_interface::RobotState<NB_ACTUATORS/2> &robot_state)
{
  
  O8O::States<NB_ACTUATORS,O8O_pam::ActuatorState> states;

  int dof;
  for(unsigned int actuator=0;actuator<NB_ACTUATORS;actuator++)
    {
      dof = actuator/2;
      if(actuator%2==0)
	{
	  states.values[actuator].set(robot_state.get_joint_state(dof).agonist);
	}
      else
	{
	  states.values[actuator].set(robot_state.get_joint_state(dof).antagonist);
	}
    }

  return states;

}

template<int QUEUE_SIZE,int NB_ACTUATORS,class DRIVER>
pam_interface::PressureAction<NB_ACTUATORS>
Standalone<QUEUE_SIZE,NB_ACTUATORS,DRIVER>::convert(const O8O::States<NB_ACTUATORS,
						O8O_pam::ActuatorState> &states)
{

  pam_interface::PressureAction<NB_ACTUATORS> action;

  int dof;
  
  for(unsigned int actuator=0;actuator<NB_ACTUATORS;actuator++)
    {
      action.set(actuator,
		 states.values[actuator].get());
    }

  return action;
 
}

template<int QUEUE_SIZE,int NB_ACTUATORS,class DRIVER>
void
Standalone<QUEUE_SIZE,NB_ACTUATORS,DRIVER>::enrich_extended_state( pam_interface::RobotState<NB_ACTUATORS/2> &extended_state,
							       const pam_interface::RobotState<NB_ACTUATORS/2> &ri_observation)
{
  extended_state = ri_observation;
}



