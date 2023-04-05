#include <memory>
#include <tuple>

#include "o80/front_end.hpp"
#include "o80/pybind11_helper.hpp"
#include "o80/state1d.hpp"
#include "o80/state2d.hpp"
#include "o80/void_extended_state.hpp"
#include "o80_pam/actuator_state.hpp"
#include "o80_pam/dummy_driver.hpp"
#include "o80_pam/pamy1_driver.hpp"
#include "o80_pam/pamy2_driver.hpp"
#include "o80_pam/robot_fk_extended_state.hpp"
#include "o80_pam/standalone.hpp"

#define NB_DOFS 4
#define QUEUE_SIZE 5000000

// pressures action sent to the robot_interfaces backend
typedef pam_interface::PressureAction<NB_DOFS * 2> PressureAction;

// pressures red from the robot interfaces backend
typedef pam_interface::RobotState<NB_DOFS> RobotState;

// drivers used by the robot_interfaces backend
typedef o80_pam::DummyDriver<NB_DOFS> DummyDriver;

// drivers used by the robot_interfaces backend
typedef o80_pam::Pamy1Driver<NB_DOFS> Pamy1Driver;

// drivers used by the robot_interfaces backend
typedef o80_pam::Pamy2Driver<NB_DOFS> Pamy2Driver;

// used as argument to the driver
typedef pam_interface::Configuration<NB_DOFS> Configuration;

// state of each actuator that will be encapsulated
// in o80::Observation
typedef o80_pam::ActuatorState ActuatorState;

// o80::States
typedef o80::States<NB_DOFS*2, ActuatorState> States;

// o80 Standalone class
typedef o80_pam::Standalone<QUEUE_SIZE, NB_DOFS * 2, DummyDriver>
    DummyStandalone;
typedef o80_pam::Standalone<QUEUE_SIZE, NB_DOFS * 2, Pamy1Driver> Pamy1Standalone;
typedef o80_pam::Standalone<QUEUE_SIZE, NB_DOFS * 2, Pamy2Driver> Pamy2Standalone;

// add the bindings to o80::Observation
// (with extra functions compared to the native o80 wrappers)
void add_observation_and_serializer(pybind11::module& m)
{
    typedef o80::Observation<2 * NB_DOFS,
                             o80_pam::ActuatorState,
                             pam_interface::RobotState<NB_DOFS>>
        observation;
    pybind11::class_<observation>(m, "Observation")
        .def(pybind11::init<>())
        .def(pybind11::init< States, States,
	                     pam_interface::RobotState<NB_DOFS>,
	                     long int, long int, double >() )
        .def("get_observed_states", &observation::get_observed_states)
        .def("get_desired_states", &observation::get_desired_states)
        .def("get_desired_pressures",
             [](observation& obs)
             {
                 o80::States<NB_DOFS * 2, ActuatorState> desired =
                     obs.get_desired_states();
                 std::array<std::tuple<int, int>, NB_DOFS> pressures;
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     int ago = desired.get(2 * dof).get();
                     int antago = desired.get(2 * dof + 1).get();
                     pressures[dof] = std::make_tuple(ago, antago);
                 }
                 return pressures;
             })
        .def("get_observed_pressures",
             [](observation& obs)
             {
                 o80::States<NB_DOFS * 2, ActuatorState> observed =
                     obs.get_observed_states();
                 std::array<std::tuple<int, int>, NB_DOFS> pressures;
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     int ago = observed.get(2 * dof).get();
                     int antago = observed.get(2 * dof + 1).get();
                     pressures[dof] = std::make_tuple(ago, antago);
                 }
                 return pressures;
             })
        .def("get_positions",
             [](observation& obs)
             {
                 const pam_interface::RobotState<NB_DOFS>& robot =
                     obs.get_extended_state();
                 std::array<double, NB_DOFS> positions;
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     positions[dof] = robot.get_position(dof);
                 }
                 return positions;
             })
        .def("get_velocities",
             [](observation& obs)
             {
                 const pam_interface::RobotState<NB_DOFS>& robot =
                     obs.get_extended_state();
                 std::array<double, NB_DOFS> velocities;
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     velocities[dof] = robot.get_velocity(dof);
                 }
                 return velocities;
             })
        .def("get_references_found",
             [](observation& obs)
             {
                 const pam_interface::RobotState<NB_DOFS>& robot =
                     obs.get_extended_state();
                 std::array<bool, NB_DOFS> ref_founds;
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     ref_founds[dof] = robot.get_reference_found(dof);
                 }
                 return ref_founds;
             })
        .def("get_extended_state", &observation::get_extended_state)
        .def("get_iteration", &observation::get_iteration)
        .def("get_frequency", &observation::get_frequency)
        .def("get_time_stamp", &observation::get_time_stamp)
        .def("__str__",
             [](const observation& o)
             {
                 // extracting all data from the observation
                 std::stringstream stream;
                 const pam_interface::RobotState<NB_DOFS>& robot =
                     o.get_extended_state();
                 o80::States<NB_DOFS * 2, ActuatorState> observed =
                     o.get_observed_states();
                 o80::States<NB_DOFS * 2, ActuatorState> desired =
                     o.get_desired_states();
                 long int iteration = o.get_iteration();
                 long int time_stamp = o.get_time_stamp();
                 double frequency = o.get_frequency();
                 // creating the string
                 stream << "Observation. Iteration: " << iteration
                        << " (frequency: " << frequency
                        << " time stamp: " << time_stamp << ")\n";
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     int ago_desired = desired.get(2 * dof).get();
                     int antago_desired = desired.get(2 * dof + 1).get();
                     int ago_observed = observed.get(2 * dof).get();
                     int antago_observed = observed.get(2 * dof + 1).get();
                     double position = robot.get_position(dof);
                     double velocity = robot.get_velocity(dof);
                     stream << "dof " << dof << "\t" << ago_observed << " ("
                            << ago_desired << ") " << antago_observed << " ("
                            << antago_desired << ") "
                            << "position: " << position
                            << " velocity: " << velocity;
                     stream << "\n";
                 }
                 return stream.str();
             });

    typedef shared_memory::Serializer<observation> serializer;
    pybind11::class_<serializer>(m, "Serializer")
        .def(pybind11::init<>())
        .def("serializable_size", &serializer::serializable_size)
        .def("serialize",
             [](serializer& s, const observation& o)
             {
                 // see:
                 // https://pybind11.readthedocs.io/en/stable/advanced/cast/strings.html
                 // "return c++ strings without conversion"
                 std::string ser = s.serialize(o);
                 return pybind11::bytes(ser);
             })
        .def("deserialize",
             [](serializer& s, const std::string& serialized)
             {
                 observation o;
                 s.deserialize(serialized, o);
                 return o;
             });
}

// add the bindings to o80::FrontEnd
// (with extra functions compared to the native o80 wrappers)
void add_frontend(pybind11::module& m)
{
    typedef o80::Observation<2 * NB_DOFS,
                             o80_pam::ActuatorState,
                             pam_interface::RobotState<NB_DOFS>>
        observation;

    typedef o80::FrontEnd<QUEUE_SIZE,
                          NB_DOFS * 2,
                          o80_pam::ActuatorState,
                          pam_interface::RobotState<NB_DOFS>>
        frontend;
    pybind11::class_<frontend>(m, "FrontEnd")
        // generic frontend bindings (similar to what o80::pybind11_helper.hpp
        // creates)
        .def(pybind11::init<std::string>())
        .def("get_frequency", &frontend::get_frequency)
        .def("get_nb_actuators", &frontend::get_nb_actuators)
        .def("get_observations_since", &frontend::get_observations_since)
        .def("get_latest_observations", &frontend::get_latest_observations)
        .def("wait_for_next", &frontend::wait_for_next)
        .def("reset_next_index", &frontend::reset_next_index)
        .def("purge", &frontend::purge)
        .def("initial_states", &frontend::initial_states)
        .def("add_command",
             (void(frontend::*)(
                 int, o80_pam::ActuatorState, o80::Iteration, o80::Mode)) &
                 frontend::add_command)
        .def("add_command",
             (void(frontend::*)(
                 int, o80_pam::ActuatorState, o80::Duration_us, o80::Mode)) &
                 frontend::add_command)
        .def("add_command",
             (void(frontend::*)(int, o80_pam::ActuatorState, o80::Mode)) &
                 frontend::add_command)
        .def("add_command",
             (void(frontend::*)(
                 int, o80_pam::ActuatorState, o80::Speed, o80::Mode)) &
                 frontend::add_command)
        .def("add_reinit_command", &frontend::add_reinit_command)
        .def("burst", &frontend::burst)
        .def("final_burst", &frontend::final_burst)
        .def("pulse_and_wait", &frontend::pulse_and_wait)
        .def("read", &frontend::read)
        .def("latest", [](frontend& fe) { return fe.read(-1); })
        .def("pulse",
             (observation(frontend::*)(o80::Iteration)) & frontend::pulse)
        .def("pulse", (observation(frontend::*)()) & frontend::pulse)

        .def("add_command",
             [](frontend& fe,
                int actuator,
                int pressure,
                o80::Iteration it,
                o80::Mode mode) {
                 fe.add_command(
                     actuator, o80_pam::ActuatorState(pressure), it, mode);
             })

        .def("add_command",
             [](frontend& fe,
                int actuator,
                int pressure,
                o80::Speed speed,
                o80::Mode mode) {
                 fe.add_command(
                     actuator, o80_pam::ActuatorState(pressure), speed, mode);
             })

        .def(
            "add_command",
            [](frontend& fe,
               int actuator,
               int pressure,
               o80::Duration_us duration,
               o80::Mode mode) {
                fe.add_command(
                    actuator, o80_pam::ActuatorState(pressure), duration, mode);
            })

        .def("add_command",
             [](frontend& fe, int actuator, int pressure, o80::Mode mode) {
                 fe.add_command(
                     actuator, o80_pam::ActuatorState(pressure), mode);
             })

        .def("add_command",
             [](frontend& fe,
                int dof,
                int ago,
                int antago,
                o80::Iteration it,
                o80::Mode mode)
             {
                 fe.add_command(2 * dof, o80_pam::ActuatorState(ago), it, mode);
                 fe.add_command(
                     2 * dof + 1, o80_pam::ActuatorState(antago), it, mode);
             })
        .def("add_command",
             [](frontend& fe,
                int dof,
                int ago,
                int antago,
                o80::Duration_us d,
                o80::Mode mode)
             {
                 fe.add_command(2 * dof, o80_pam::ActuatorState(ago), d, mode);
                 fe.add_command(
                     2 * dof + 1, o80_pam::ActuatorState(antago), d, mode);
             })
        .def("add_command",
             [](frontend& fe,
                int dof,
                int ago,
                int antago,
                o80::Speed s,
                o80::Mode mode)
             {
                 fe.add_command(2 * dof, o80_pam::ActuatorState(ago), s, mode);
                 fe.add_command(
                     2 * dof + 1, o80_pam::ActuatorState(antago), s, mode);
             })
        .def("add_command",
             [](frontend& fe, int dof, int ago, int antago, o80::Mode mode)
             {
                 fe.add_command(2 * dof, o80_pam::ActuatorState(ago), mode);
                 fe.add_command(
                     2 * dof + 1, o80_pam::ActuatorState(antago), mode);
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Speed s,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2 * dof, o80_pam::ActuatorState(ago[dof]), s, mode);
                     fe.add_command(2 * dof + 1,
                                    o80_pam::ActuatorState(antago[dof]),
                                    s,
                                    mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Iteration it,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2 * dof, o80_pam::ActuatorState(ago[dof]), it, mode);
                     fe.add_command(2 * dof + 1,
                                    o80_pam::ActuatorState(antago[dof]),
                                    it,
                                    mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Duration_us d,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2 * dof, o80_pam::ActuatorState(ago[dof]), d, mode);
                     fe.add_command(2 * dof + 1,
                                    o80_pam::ActuatorState(antago[dof]),
                                    d,
                                    mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2 * dof, o80_pam::ActuatorState(ago[dof]), mode);
                     fe.add_command(2 * dof + 1,
                                    o80_pam::ActuatorState(antago[dof]),
                                    mode);
                 }
             });
}

// add the bindings to o80::Observation corresponding to ball, targets,
// hit points (etc).
void add_mirror_free_joint_observation_and_serializer(pybind11::module& m)
{
    typedef o80::Observation<6, o80::State1d, o80::VoidExtendedState>
        observation;
    pybind11::class_<observation>(m, "MirrorFreeJointObservation")
        .def(pybind11::init<>())
        .def("get_position",
             [](const observation& o)
             {
                 o80::States<6, o80::State1d> observed =
                     o.get_observed_states();
                 std::array<double, 3> position;
                 for (int dim = 0; dim < 3; dim++)
                 {
                     position[dim] = observed.get(2 * dim).get();
                 }
                 return position;
             })
        .def("get_velocity",
             [](const observation& o)
             {
                 o80::States<6, o80::State1d> observed =
                     o.get_observed_states();
                 std::array<double, 3> velocity;
                 for (int dim = 0; dim < 3; dim++)
                 {
                     velocity[dim] = observed.get(2 * dim + 1).get();
                 }
                 return velocity;
             })
        .def("get_iteration", &observation::get_iteration)
        .def("get_frequency", &observation::get_frequency)
        .def("get_time_stamp", &observation::get_time_stamp)
        .def("get_observed_states", &observation::get_observed_states)
        .def("get_desired_states", &observation::get_desired_states)
        .def("__str__",
             [](const observation& o)
             {
                 // extracting all data from the observation
                 std::stringstream stream;
                 o80::States<6, o80::State1d> observed =
                     o.get_observed_states();
                 std::array<double, 3> position;
                 std::array<double, 3> velocity;
                 for (int dim = 0; dim < 3; dim++)
                 {
                     position[dim] = observed.get(2 * dim).get();
                     velocity[dim] = observed.get(2 * dim + 1).get();
                 }

                 long int iteration = o.get_iteration();
                 long int time_stamp = o.get_time_stamp();
                 double frequency = o.get_frequency();
                 // creating the string
                 stream << "Observation. Iteration: " << iteration
                        << " (frequency: " << frequency
                        << " time stamp: " << time_stamp << ")\n"
                        << "position: ";
                 for (uint dim = 0; dim < 3; dim++)
                 {
                     stream << position[dim] << ", ";
                 }
                 stream << "\nvelocity: ";
                 for (uint dim = 0; dim < 3; dim++)
                 {
                     stream << velocity[dim] << ", ";
                 }
                 stream << "\n";
                 return stream.str();
             });

    typedef shared_memory::Serializer<observation> serializer;
    pybind11::class_<serializer>(m, "MirrorFreeJointSerializer")
        .def(pybind11::init<>())
        .def("serializable_size", &serializer::serializable_size)
        .def("serialize",
             [](serializer& s, const observation& o)
             {
                 // see:
                 // https://pybind11.readthedocs.io/en/stable/advanced/cast/strings.html
                 // "return c++ strings without conversion"
                 std::string ser = s.serialize(o);
                 return pybind11::bytes(ser);
             })
        .def("deserialize",
             [](serializer& s, const std::string& serialized)
             {
                 observation o;
                 s.deserialize(serialized, o);
                 return o;
             });
}

// add the bindings to o80::FrontEnd
// (with extra functions compared to the native o80 wrappers)
void add_mirror_free_joint_frontend(pybind11::module& m)
{
    typedef o80::Observation<6, o80::State1d, o80::VoidExtendedState>
        observation;
    typedef o80::FrontEnd<QUEUE_SIZE, 6, o80::State1d, o80::VoidExtendedState>
        frontend;
    pybind11::class_<frontend>(m, "MirrorFreeJointFrontEnd")
        // generic frontend bindings (similar to what o80::pybind11_helper.hpp
        // creates)
        .def(pybind11::init<std::string>())
        .def("get_nb_actuators", &frontend::get_nb_actuators)
        .def("get_observations_since", &frontend::get_observations_since)
        .def("get_latest_observations", &frontend::get_latest_observations)
        .def("wait_for_next", &frontend::wait_for_next)
        .def("reset_next_index", &frontend::reset_next_index)
        .def("initial_states", &frontend::initial_states)
        .def("burst", &frontend::burst)
        .def("purge", &frontend::purge)
        .def("final_burst", &frontend::final_burst)
        .def("pulse_and_wait", &frontend::pulse_and_wait)
        .def("read", &frontend::read)
        .def("latest", [](frontend& fe) { return fe.read(-1); })
        .def("pulse",
             (observation(frontend::*)(o80::Iteration)) & frontend::pulse)
        .def("pulse", (observation(frontend::*)()) & frontend::pulse)
        .def("add_reinit_command", &frontend::add_reinit_command)
        .def("add_command",
             [](frontend& fe,
                std::array<double, 3> position,
                std::array<double, 3> velocity,
                o80::Iteration it,
                o80::Mode mode)
             {
                 for (uint dim = 0; dim < 3; dim++)
                 {
                     fe.add_command(
                         2 * dim, o80::State1d(position[dim]), it, mode);
                     fe.add_command(
                         2 * dim + 1, o80::State1d(velocity[dim]), it, mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<double, 3> position,
                std::array<double, 3> velocity,
                o80::Speed speed,
                o80::Mode mode)
             {
                 for (uint dim = 0; dim < 3; dim++)
                 {
                     fe.add_command(
                         2 * dim, o80::State1d(position[dim]), speed, mode);
                     fe.add_command(
                         2 * dim + 1, o80::State1d(velocity[dim]), speed, mode);
                 }
             })

        .def("add_command",
             [](frontend& fe,
                std::array<double, 3> position,
                std::array<double, 3> velocity,
                o80::Duration_us duration,
                o80::Mode mode)
             {
                 for (uint dim = 0; dim < 3; dim++)
                 {
                     fe.add_command(
                         2 * dim, o80::State1d(position[dim]), duration, mode);
                     fe.add_command(2 * dim + 1,
                                    o80::State1d(velocity[dim]),
                                    duration,
                                    mode);
                 }
             })

        .def("add_command",
             [](frontend& fe,
                std::array<double, 3> position,
                std::array<double, 3> velocity,
                o80::Mode mode)
             {
                 for (uint dim = 0; dim < 3; dim++)
                 {
                     fe.add_command(2 * dim, o80::State1d(position[dim]), mode);
                     fe.add_command(
                         2 * dim + 1, o80::State1d(velocity[dim]), mode);
                 }
             });
}

// add the bindings to o80::Observation corresponding robot mirroring,
// i.e. sending joint position and velocities to robot
void add_mirror_robot_observation_and_serializer(pybind11::module& m)
{
    pybind11::class_<o80_pam::RobotFKExtendedState>(m, "RobotFKExtendedState")
        .def(pybind11::init<>())
        .def(pybind11::init<const std::array<double, 3>&,
                            const std::array<double, 9>&>())
        .def("set_position", &o80_pam::RobotFKExtendedState::set_position)
        .def("set_orientation", &o80_pam::RobotFKExtendedState::set_orientation)
        .def("get_position", &o80_pam::RobotFKExtendedState::get_position)
        .def("get_orientation",
             &o80_pam::RobotFKExtendedState::get_orientation);

    typedef o80::
        Observation<NB_DOFS, o80::State2d, o80_pam::RobotFKExtendedState>
            observation;
    pybind11::class_<observation>(m, "MirrorRobotObservation")
        .def(pybind11::init<>())
        .def("get_positions",
             [](const observation& o)
             {
                 o80::States<NB_DOFS, o80::State2d> observed =
                     o.get_observed_states();
                 std::array<double, NB_DOFS> positions;
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     positions[dof] = observed.get(dof).get<0>();
                 }
                 return positions;
             })
        .def("get_velocities",
             [](const observation& o)
             {
                 o80::States<NB_DOFS, o80::State2d> observed =
                     o.get_observed_states();
                 std::array<double, NB_DOFS> velocities;
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     velocities[dof] = observed.get(dof).get<1>();
                 }
                 return velocities;
             })
        .def("get_cartesian_position",
             [](const observation& o)
             {
                 o80_pam::RobotFKExtendedState fk = o.get_extended_state();
                 return fk.get_position();
             })
        .def("get_cartesian_orientation",
             [](const observation& o)
             {
                 o80_pam::RobotFKExtendedState fk = o.get_extended_state();
                 return fk.get_orientation();
             })
        .def("get_observed_states", &observation::get_observed_states)
        .def("get_desired_states", &observation::get_desired_states)
        .def("get_iteration", &observation::get_iteration)
        .def("get_frequency", &observation::get_frequency)
        .def("get_time_stamp", &observation::get_time_stamp)
        .def("__str__",
             [](const observation& o)
             {
                 // extracting all data from the observation
                 std::stringstream stream;
                 o80::States<NB_DOFS, o80::State2d> observed =
                     o.get_observed_states();
                 std::array<double, NB_DOFS> position;
                 std::array<double, NB_DOFS> velocity;
                 for (int dof = 0; dof < 3; dof++)
                 {
                     position[dof] = observed.get(dof).get<0>();
                     velocity[dof] = observed.get(dof).get<1>();
                 }
                 long int iteration = o.get_iteration();
                 long int time_stamp = o.get_time_stamp();
                 double frequency = o.get_frequency();
                 // creating the string
                 stream << "Observation. Iteration: " << iteration
                        << " (frequency: " << frequency
                        << " time stamp: " << time_stamp << ")\n"
                        << "position: ";
                 for (uint dof = 0; dof < 3; dof++)
                 {
                     stream << position[dof] << ", ";
                 }
                 stream << "\nvelocity: ";
                 for (uint dof = 0; dof < 3; dof++)
                 {
                     stream << velocity[dof] << ", ";
                 }
                 stream << "\n";
                 return stream.str();
             });

    typedef shared_memory::Serializer<observation> serializer;
    pybind11::class_<serializer>(m, "MirrorRobotSerializer")
        .def(pybind11::init<>())
        .def("serializable_size", &serializer::serializable_size)
        .def("serialize",
             [](serializer& s, const observation& o)
             {
                 // see:
                 // https://pybind11.readthedocs.io/en/stable/advanced/cast/strings.html
                 // "return c++ strings without conversion"
                 std::string ser = s.serialize(o);
                 return pybind11::bytes(ser);
             })
        .def("deserialize",
             [](serializer& s, const std::string& serialized)
             {
                 observation o;
                 s.deserialize(serialized, o);
                 return o;
             });
}

// add the bindings to o80::FrontEnd
// (with extra functions compared to the native o80 wrappers)
void add_mirror_robot_frontend(pybind11::module& m)
{
    typedef o80::
        Observation<NB_DOFS, o80::State2d, o80_pam::RobotFKExtendedState>
            observation;
    typedef o80::FrontEnd<QUEUE_SIZE,
                          NB_DOFS,
                          o80::State2d,
                          o80_pam::RobotFKExtendedState>
        frontend;
    pybind11::class_<frontend>(m, "MirrorRobotFrontEnd")
        // generic frontend bindings (similar to what o80::pybind11_helper.hpp
        // creates)
        .def(pybind11::init<std::string>())
        .def("get_nb_actuators", &frontend::get_nb_actuators)
        .def("get_observations_since", &frontend::get_observations_since)
        .def("get_latest_observations", &frontend::get_latest_observations)
        .def("wait_for_next", &frontend::wait_for_next)
        .def("reset_next_index", &frontend::reset_next_index)
        .def("initial_states", &frontend::initial_states)
        .def("burst", &frontend::burst)
        .def("purge", &frontend::purge)
        .def("final_burst", &frontend::final_burst)
        .def("pulse_and_wait", &frontend::pulse_and_wait)
        .def("read", &frontend::read)
        .def("latest", [](frontend& fe) { return fe.read(-1); })
        .def("pulse",
             (observation(frontend::*)(o80::Iteration)) & frontend::pulse)
        .def("pulse", (observation(frontend::*)()) & frontend::pulse)
        .def("add_reinit_command", &frontend::add_reinit_command)
        .def("add_command",
             [](frontend& fe,
                std::array<double, NB_DOFS> position,
                std::array<double, NB_DOFS> velocity,
                o80::Iteration it,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < 4; dof++)
                 {
                     fe.add_command(dof,
                                    o80::State2d(position[dof], velocity[dof]),
                                    it,
                                    mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<double, 4> position,
                std::array<double, 4> velocity,
                o80::Speed speed,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < 4; dof++)
                 {
                     fe.add_command(dof,
                                    o80::State2d(position[dof], velocity[dof]),
                                    speed,
                                    mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<double, 4> position,
                std::array<double, 4> velocity,
                o80::Duration_us duration,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < 4; dof++)
                 {
                     fe.add_command(dof,
                                    o80::State2d(position[dof], velocity[dof]),
                                    duration,
                                    mode);
                 }
             })

        .def("add_command",
             [](frontend& fe,
                std::array<double, 4> position,
                std::array<double, 4> velocity,
                o80::Mode mode)
             {
                 for (uint dof = 0; dof < 4; dof++)
                 {
                     fe.add_command(
                         dof, o80::State2d(position[dof], velocity[dof]), mode);
                 }
             });
}

PYBIND11_MODULE(o80_pam_wrp, m)
{
    // core bindings, common to dummy and real
    o80::create_python_bindings<
        QUEUE_SIZE,
        2 * NB_DOFS,
        ActuatorState,
        RobotState,
        o80::NO_EXTENDED_STATE,  // RobotState, already binded by pam_interface
        o80::NO_OBSERVATION,     // added below
        o80::NO_FRONTEND,        // added below>
        o80::NO_SERIALIZER>      // added below
        (m);

    add_observation_and_serializer(m);
    add_frontend(m);

    // wrappers for dummy robot
    std::string prefix_dummy("dummy_");
    o80::create_standalone_python_bindings<
        DummyDriver,
        DummyStandalone,
        pam_interface::Configuration<NB_DOFS>>(m, prefix_dummy);

    // wrappers for pamy1 robot
    std::string prefix_pamy1("pamy1_");
    o80::create_standalone_python_bindings<
        Pamy1Driver,
        Pamy1Standalone,
        pam_interface::Configuration<NB_DOFS>>(m, prefix_pamy1);

   // wrappers for pamy1 robot
    std::string prefix_pamy2("pamy2_");
    o80::create_standalone_python_bindings<
        Pamy2Driver,
        Pamy2Standalone,
        pam_interface::Configuration<NB_DOFS>,
        std::string,
        unsigned int>(m, prefix_pamy2);

    // extra o80 wrappers for exchange of "mirroring" information,
    // i.e. position and velocity for each joint
    o80::create_python_bindings<QUEUE_SIZE,
                                NB_DOFS,
                                o80::State2d,  // 1 DOFS: position + velocity
                                o80_pam::RobotFKExtendedState,
                                o80::NO_STATE,  // o80::State2d already binded
                                                // in package o80
                                o80::NO_EXTENDED_STATE,  // added_below
                                o80::NO_OBSERVATION,     // added_below
                                o80::NO_SERIALIZER,      // added below
                                o80::NO_FRONTEND>        // added below
        (m, std::string("MirrorRobot"));
    add_mirror_robot_frontend(m);
    add_mirror_robot_observation_and_serializer(m);

    // extra o80 wrappers for exchange of information regarding
    // a 6d object (3d position + 3d velocity), e.g. a ball in table
    // tennis settings.
    // 6 : position3d + velocity3d
    o80::create_python_bindings<QUEUE_SIZE,
                                6,
                                o80::State1d,
                                o80::VoidExtendedState,
                                o80::NO_OBSERVATION,  // added below
                                o80::NO_SERIALIZER,   // added below
                                o80::NO_STATE,  // o80::State2d already binded
                                                // in package o80
                                o80::NO_EXTENDED_STATE,
                                o80::NO_FRONTEND>  // added below
        (m, std::string("MirrorFreeJoint"));
    add_mirror_free_joint_observation_and_serializer(m);
    add_mirror_free_joint_frontend(m);
}
