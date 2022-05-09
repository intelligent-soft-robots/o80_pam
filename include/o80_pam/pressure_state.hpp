#pragma once

#include "o80/state_helper.hpp"
#include "shared_memory/serializer.hpp"

namespace o80_pam
{
class PressureState
{
public:
    PressureState();
    PressureState(int pressure);

    static PressureState eval(int value);

    std::string to_string() const;

    void set(int value);
    int get() const;

    bool finished(const TimePoint &start_time,
                    const TimePoint &now,
                    const PressureState &start_state,
                    const PressureState &current_state,
                    const PressureState &target_state,
                    double speed) const;

    PressureState intermediate_state(long int iteration_start,
                                        long int iteration_now,
                                        const PressureState &start_state,
                                        const PressureState &current_state,
                                        const PressureState &target_state,
                                        const Iteration &iteration) const;

    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(pressure_);
    }

private:
    friend shared_memory::private_serialization;
    int pressure_;
}
}  // namespace :
