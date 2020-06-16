#pragma once

#include "o80/back_end.hpp"
#include "o80/observation.hpp"
#include "pam_interface/pressure_action.hpp"
#include "pam_interface/state/joint.hpp"
#include "pam_interface/state/robot.hpp"
#include "o80_pam/actuator_state.hpp"

namespace o80_pam
{
// Driver will be DummyRobotDriver or RealRobotDriver
template <int QUEUE_SIZE, int NB_ACTUATORS, class Driver>
class Standalone
    : public o80::Standalone<QUEUE_SIZE,
                             NB_ACTUATORS,
			     Driver,
                             o80_pam::ActuatorState,
                             pam_interface::RobotState<NB_ACTUATORS / 2>>
{
public:
    typedef std::shared_ptr<Driver> DriverPtr;

    Standalone(DriverPtr &ri_driver, double frequency, std::string segment_id);

    ~Standalone();

    o80::States<NB_ACTUATORS, o80_pam::ActuatorState> convert(
        const pam_interface::RobotState<NB_ACTUATORS / 2> &robot_state);

    pam_interface::PressureAction<NB_ACTUATORS> convert(
        const o80::States<NB_ACTUATORS, o80_pam::ActuatorState> &states);

    void enrich_extended_state(
        pam_interface::RobotState<NB_ACTUATORS / 2> &extended_state,
        const pam_interface::RobotState<NB_ACTUATORS / 2> &ri_observation);
};

#include "standalone.hxx"
}
