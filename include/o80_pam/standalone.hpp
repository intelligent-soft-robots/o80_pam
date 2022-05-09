#pragma once

#include "o80/back_end.hpp"
#include "o80/observation.hpp"
#include "o80_pam/actuator_state.hpp"
#include "pam_interface/pressure_action.hpp"
#include "pam_interface/state/joint.hpp"
#include "pam_interface/state/robot.hpp"

namespace o80_pam
{

/**
 * @brief Standalone class.
 * 
 * Standalone class can run without hardware environment.
 * According to specified parameters Standalone can be used
 * for either Dummy Robot Driver or Real Robot Driver like
 * e.g. Pamy 1 or Pamy 2.
 * 
 * @tparam QUEUE_SIZE 
 * @tparam NB_ACTUATORS number of actuators
 * @tparam Driver system specific driver
 */
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

    /**
     * Constructor of Standalone object.
     * 
     * @param ri_driver 
     * @param frequency 
     * @param segment_id 
     */
    Standalone(DriverPtr &ri_driver, double frequency, std::string segment_id);

    /**
     * Destructor Standalone object.
     */
    ~Standalone();

    /**
     * @brief 
     * 
     * @param robot_state 
     * @return o80::States<NB_ACTUATORS, o80_pam::ActuatorState> 
     */
    o80::States<NB_ACTUATORS, o80_pam::ActuatorState> convert(
        const pam_interface::RobotState<NB_ACTUATORS / 2> &robot_state);

    /**
     * @brief 
     * 
     * @param states 
     * @return pam_interface::PressureAction<NB_ACTUATORS> 
     */
    pam_interface::PressureAction<NB_ACTUATORS> convert(
        const o80::States<NB_ACTUATORS, o80_pam::ActuatorState> &states);

    /**
     * @brief 
     * 
     * @param extended_state 
     * @param ri_observation 
     */
    void enrich_extended_state(
        pam_interface::RobotState<NB_ACTUATORS / 2> &extended_state,
        const pam_interface::RobotState<NB_ACTUATORS / 2> &ri_observation);
};

#include "standalone.hxx"
}  // namespace o80_pam
