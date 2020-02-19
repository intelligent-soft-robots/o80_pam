
template<int QUEUE_SIZE,int NB_ACTUATORS>
Standalone<QUEUE_SIZE,NB_ACTUATORS>::Standalone(DriverPtr ri_driver
						&ri_driver,
						double frequency,
						std::string segment_id,
						std::string object_id)
  : O8O::Standalone<QUEUE_SIZE,
		    NB_ACTUATORS,
		    pam_interface::PressureAction<NB_ACTUATORS>,
		    pam_interface::RobotState<NB_ACTUATORS/2>,
		    O8O_pam::ActuatorState,
		    pam_interface::RobotState<NB_ACTUATORS/2> >
  (ri_driver,
   std::numeric_limits<double>::infinity(), // max_action_duration_s
   std::numeric_limits<double>::infinity(), // max_inter_action_duration_s
   frequency,
   segment_id,
   object_id,
   simulation)
{
}


template<int QUEUE_SIZE,int NB_ACTUATORS>
Standalone<QUEUE_SIZE,NB_ACTUATORS>::~Standalone()
{}


template<int QUEUE_SIZE,int NB_ACTUATORS>
O8O::States<NB_ACTUATORS,
	    O8O_pam::ActuatorState>
Standalone<QUEUE_SIZE,NB_ACTUATORS>::convert(const pam_interface::RobotState<NB_ACTUATORS/2> &robot_state)
{
  
  O8O::States<NB_ACTUATORS,O8O_pam::ActuatorState> states;

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
Standalone<QUEUE_SIZE,NB_ACTUATORS>::convert(const O8O::States<NB_ACTUATORS,
						O8O_pam::ActuatorState> &states)
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

template<int QUEUE_SIZE,int NB_ACTUATORS>
void
Standalone<QUEUE_SIZE,NB_ACTUATORS>::enrich_extended_state( pam_interface::RobotState<NB_ACTUATORS/2> &extended_state,
							       const pam_interface::RobotState<NB_ACTUATORS/2> &ri_observation)
{
  extended_state = ri_observation;
}



