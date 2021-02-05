#pragma once

namespace O8O_pam
{
template <int NB_DOFS>
class ExtendedState
{
public:
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(frequency, iteration);
    }
    double frequency;
    int iteration;
    std::array<int, NB_DOFS> encoders;
    std::array<double, NB_DOFS> positions;
    std::array<double, NB_DOFS> velocities;
    std::array<bool, NB_DOFS> references_found;
};
}  // namespace O8O_pam
