// Copyright (c) 2021 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <array>
#include "shared_memory/serializer.hpp"

namespace o80_pam
{

/** 
 * RobotFKExtendedState encapsulates the forward kinematics of the 
 * robot (position, velocity and orientation of the racket).
 * 
 * It is meant to be used as ExtendedState in a o80 observation 
 * (i.e. supplementary data that can be added to an observation). 
 * 
 * See pam_mujoco/mirror_robot.hpp for an example of the FK data 
 * of the simulated robot is added to an (o80) observation
 */
class RobotFKExtendedState
{
public:
    /**
     * Constructor of Robot Forward Kinematics Extended State object.
     */
    RobotFKExtendedState();

    /**
     * Constructor of Robot Forward Kinematics Extended State object.
     * 
     * @param position 
     * @param orientation 
     */
    RobotFKExtendedState(const std::array<double, 3>& position,
                         const std::array<double, 9>& orientation);

    /**
     * Sets the position of specified dimension with given value.
     * 
     * @param dim dimension
     * @param value desired position value
     */
    void set_position(int dim, double value);

    /**
     * Sets the orientation of specified dimension with given value.
     * 
     * @param dim dimension
     * @param value desired orientation value
     */
    void set_orientation(int dim, double value);

    /**
     * Gets the extended state position as std::array with 3 values for
     * each 3D coordinate.
     * 
     * @return const std::array<double, 3>& constant reference to position
     * value as std::array
     */
    const std::array<double, 3>& get_position();

    /**
     * Gets the extended state orientation as std::array with 9 values
     * for each pair of 3D coordinates.
     * 
     * @return const std::array<double, 9>& constant reference to 
     * orientation value as std::array
     */
    const std::array<double, 9>& get_orientation();

    /**
     * Archive class for storing position and orientation values.
     * 
     * @tparam Archive template type
     * @param archive reference to archive object.
     */
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(position_, orientation_);
    }

private:
    friend shared_memory::private_serialization;
    std::array<double, 3> position_;
    std::array<double, 9> orientation_;
};

}  // namespace o80_pam
