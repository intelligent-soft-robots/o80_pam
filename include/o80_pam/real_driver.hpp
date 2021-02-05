#pragma once

#include "o80/driver.hpp"
#include "pam_interface/real/driver.hpp"

namespace o80_pam
{
template <int NB_DOFS>
class RealDriver
    : public pam_interface::RealRobotDriver<NB_DOFS>,
      public o80::Driver<pam_interface::PressureAction<2 * NB_DOFS>,
                         pam_interface::RobotState<NB_DOFS>>
{
public:
    RealDriver(const pam_interface::Configuration<NB_DOFS>& config)
        : pam_interface::RealRobotDriver<NB_DOFS>(config)
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
