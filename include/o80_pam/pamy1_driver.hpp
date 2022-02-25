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
    Pamy1Driver(const pam_interface::Configuration<NB_DOFS>& config)
        : pam_interface::Pamy1Driver<NB_DOFS>(config)
    {
    }
    void stop()
    {
    }
    void start()
    {
    }
    void set(const pam_interface::PressureAction<2 * NB_DOFS>& pressure_action)
    {
        this->in(pressure_action);
    }
    pam_interface::RobotState<NB_DOFS> get()
    {
        return this->out();
    }
};

}  // namespace o80_pam
