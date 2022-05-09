#pragma once

#include "o80/driver.hpp"
#include "pam_interface/dummy/driver.hpp"

namespace o80_pam
{

/**
 * @brief DummyDriver class.
 * 
 * o80 driver for dummy robot. Mostly encapsulates an instance
 * of pam_interace::DummyRobotDriver.
 * 
 * @tparam NB_DOFS number of degrees of freedom
 */
template <int NB_DOFS>
class DummyDriver
    : public pam_interface::DummyRobotDriver<NB_DOFS>,
      public o80::Driver<pam_interface::PressureAction<2 * NB_DOFS>,
                         pam_interface::RobotState<NB_DOFS>>
{
public:
    /**
     * Constructs a new DummyDriver object.
     * 
     * @param config configuration object with dummy driver parameters
     */
    DummyDriver(const pam_interface::Configuration<NB_DOFS>& config)
        : pam_interface::DummyRobotDriver<NB_DOFS>(config)
    {

    }

    /**
     * Stops o80 dummy driver process.
     */
    void stop()
    {

    }
    
    /**
     *  Starts o80 dummy driver process.
     */
    void start()
    {

    }
    
    /**
     * Sets pressure command via handing a pressure action object.
     * 
     * @param pressure_action pressure value to be set
     */
    void set(const pam_interface::PressureAction<2 * NB_DOFS>& pressure_action)
    {
        this->in(pressure_action);
    }
    
    /**
     *  Gets state parameters as RobotState object via pam_interface.
     * 
     * @return pam_interface::RobotState<NB_DOFS> state parameters
     */
    pam_interface::RobotState<NB_DOFS> get()
    {
        return this->out();
    }
};

}  // namespace o80_pam
