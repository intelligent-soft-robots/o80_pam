#pragma once

#include "o80/driver.hpp"
#include "pam_interface/real/pamy2/driver.hpp"

namespace o80_pam
{
/**
 * o80 driver for the Pamy2 robot. Mostly encapsulate
 * an instance of pam_interface::Pamy2Driver.
 */
template <int NBDOFS>
class Pamy2Driver
    : public pam_interface::Pamy2Driver,
      public o80::Driver<pam_interface::PressureAction<2 * NBDOFS>,
                         pam_interface::RobotState<NBDOFS> >
{
public:
    Pamy2Driver(pam_interface::Configuration<NBDOFS>& config,
                std::string ip,
                unsigned int port)
        : pam_interface::Pamy2Driver(config, ip, port)
    {
    }
    void stop()
    {
    }
    void start()
    {
    }
    void set(const pam_interface::PressureAction<2 * NBDOFS>& pressure_action)
    {
        this->in(pressure_action);
    }
    pam_interface::RobotState<NB_DOFS> get()
    {
        return this->out();
    }
};

}  // namespace o80_pam
