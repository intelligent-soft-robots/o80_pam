#pragma once

#include "o80/driver.hpp"
#include "pam_interface/real/pamy1/driver.hpp"

namespace o80_pam
{
/**
 * o80 driver for the Pamy1 robot. Mostly encapsulate
 * an instance of pam_interface::Pamy2Driver.
 */
template <int NB_DOFS>
class Pamy1Driver
    : public pam_interface::Pamy1Driver<NB_DOFS>,
      public o80::Driver<pam_interface::PressureAction<2 * NB_DOFS>,
                         pam_interface::RobotState<NB_DOFS> >
{
public:

    /**
     * @brief Construct a new Pamy 1 Driver object
     * 
     * @param config 
     */
    Pamy1Driver(const pam_interface::Configuration<NB_DOFS>& config)
        : pam_interface::Pamy1Driver<NB_DOFS>(config)
    {
    }

    /**
     * @brief 
     * 
     */
    void stop()
    {
    }

    /**
     * @brief 
     * 
     */
    void start()
    {
    }

    /**
     * @brief 
     * 
     * @param pressure_action 
     */
    void set(const pam_interface::PressureAction<2 * NB_DOFS>& pressure_action)
    {
        this->in(pressure_action);
    }

    /**
     * @brief 
     * 
     * @return pam_interface::RobotState<NB_DOFS> 
     */
    pam_interface::RobotState<NB_DOFS> get()
    {
        return this->out();
    }
};

}  // namespace o80_pam
