
template <int QUEUE_SIZE, int NB_ACTUATORS, class DRIVER>
Standalone<QUEUE_SIZE, NB_ACTUATORS, DRIVER>::Standalone(DriverPtr &ri_driver,
                                                         double frequency,
                                                         std::string segment_id)
  : o80::Standalone<QUEUE_SIZE,
  NB_ACTUATORS,DRIVER,
  o80_pam::ActuatorState,
  pam_interface::RobotState<NB_ACTUATORS / 2>>(
					       ri_driver, frequency, segment_id)
{
}

template <int QUEUE_SIZE, int NB_ACTUATORS, class DRIVER>
Standalone<QUEUE_SIZE, NB_ACTUATORS, DRIVER>::~Standalone()
{
}

template <int QUEUE_SIZE, int NB_ACTUATORS, class DRIVER>
o80::States<NB_ACTUATORS, o80_pam::ActuatorState>
Standalone<QUEUE_SIZE, NB_ACTUATORS, DRIVER>::convert(
    const pam_interface::RobotState<NB_ACTUATORS / 2> &robot_state)
{
    o80::States<NB_ACTUATORS, o80_pam::ActuatorState> states;

    int dof;
    for (unsigned int actuator = 0; actuator < NB_ACTUATORS; actuator++)
    {
        dof = actuator / 2;
        if (actuator % 2 == 0)
        {
            states.values[actuator].set(
                robot_state.get_joint_state(dof).agonist);
        }
        else
        {
            states.values[actuator].set(
                robot_state.get_joint_state(dof).antagonist);
        }
    }

    return states;
}

template <int QUEUE_SIZE, int NB_ACTUATORS, class DRIVER>
pam_interface::PressureAction<NB_ACTUATORS>
Standalone<QUEUE_SIZE, NB_ACTUATORS, DRIVER>::convert(
    const o80::States<NB_ACTUATORS, o80_pam::ActuatorState> &states)
{
    pam_interface::PressureAction<NB_ACTUATORS> action;

    int dof;

    for (unsigned int actuator = 0; actuator < NB_ACTUATORS; actuator++)
    {
        action.set(actuator, states.values[actuator].get());
    }

    return action;
}

template <int QUEUE_SIZE, int NB_ACTUATORS, class DRIVER>
void Standalone<QUEUE_SIZE, NB_ACTUATORS, DRIVER>::enrich_extended_state(
    pam_interface::RobotState<NB_ACTUATORS / 2> &extended_state,
    const pam_interface::RobotState<NB_ACTUATORS / 2> &ri_observation)
{
    extended_state = ri_observation;
}
