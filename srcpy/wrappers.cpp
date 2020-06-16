#include <memory>
#include <tuple>

#include "pam_interface/real/driver.hpp"
#include "pam_interface/dummy/driver.hpp"
#include "o80/front_end.hpp"
#include "o80/pybind11_helper.hpp"
#include "o80_pam/actuator_state.hpp"
#include "o80_pam/standalone.hpp"

#define NB_DOFS 4
#define QUEUE_SIZE 500000

// pressures action sent to the robot_interfaces backend
typedef pam_interface::PressureAction<NB_DOFS * 2> PressureAction;

// pressures red from the robot interfaces backend
typedef pam_interface::RobotState<NB_DOFS> RobotState;

// drivers used by the robot_interfaces backend
typedef pam_interface::DummyRobotDriver<NB_DOFS> DummyRobotDriver;

// drivers used by the robot_interfaces backend
typedef pam_interface::RealRobotDriver<NB_DOFS> RealRobotDriver;

// used as argument to the driver
typedef pam_interface::Configuration<NB_DOFS> Configuration;

// state of each actuator that will be encapsulated
// in o80::Observation
typedef o80_pam::ActuatorState ActuatorState;

// o80 Standalone class
typedef o80_pam::Standalone<QUEUE_SIZE, NB_DOFS * 2, DummyRobotDriver>
    DummyStandalone;
typedef o80_pam::Standalone<QUEUE_SIZE, NB_DOFS * 2, RealRobotDriver>
    RealStandalone;


// add the bindings to o80::Observation
// (with extra functions compared to the native o80 wrappers)
void add_observation(pybind11::module& m)
{
    typedef o80::Observation<2 * NB_DOFS,
                             o80_pam::ActuatorState,
                             pam_interface::RobotState<NB_DOFS>>
        observation;
    pybind11::class_<observation>(m,"Observation")
        .def(pybind11::init<>())
        .def("get_observed_states", &observation::get_observed_states)
        .def("get_desired_states", &observation::get_desired_states)
        .def("get_desired_pressures",
             [](observation& obs) {
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
             [](observation& obs) {
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
             [](observation& obs) {
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
             [](observation& obs) {
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
             [](observation& obs) {
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
        .def("__str__", &observation::to_string);
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
    pybind11::class_<frontend>(m,"FrontEnd")
        // generic frontend bindings (similar to what o80::pybind11_helper.hpp
        // creates)
        .def(pybind11::init<std::string>())
        .def("get_nb_actuators", &frontend::get_nb_actuators)
        .def("get_observations_since", &frontend::get_observations_since)
        .def("get_latest_observations", &frontend::get_latest_observations)
        .def("wait_for_next", &frontend::wait_for_next)
        .def("reset_next_index", &frontend::reset_next_index)
        .def("add_command",
             (void (frontend::*)(
                 int, o80_pam::ActuatorState, o80::Iteration, o80::Mode)) &
                 frontend::add_command)
        .def("add_command",
             (void (frontend::*)(
                 int, o80_pam::ActuatorState, o80::Duration_us, o80::Mode)) &
                 frontend::add_command)
        .def("add_command",
             (void (frontend::*)(int, o80_pam::ActuatorState, o80::Mode)) &
                 frontend::add_command)
        .def("add_command",
             (void (frontend::*)(
                 int, o80_pam::ActuatorState, o80::Speed, o80::Mode)) &
                 frontend::add_command)
        .def("burst", &frontend::burst)
        .def("final_burst", &frontend::final_burst)
        .def("pulse_and_wait", &frontend::pulse_and_wait)
        .def("read", &frontend::read)
        .def("latest", [](frontend& fe) { return fe.read(-1); })
        .def("pulse",
             (observation (frontend::*)(o80::Iteration)) & frontend::pulse)
        .def("pulse", (observation (frontend::*)()) & frontend::pulse)

        // extra frontend bindings
      .def("add_command",
	   [](frontend& fe,
	      int actuator,
	      int pressure,
	      o80::Iteration it,
	      o80::Mode mode) {
	     fe.add_command(actuator, o80_pam::ActuatorState(pressure), it, mode);
	   })


      .def("add_command",
	   [](frontend& fe,
	      int actuator,
	      int pressure,
	      o80::Speed speed,
	      o80::Mode mode) {
	     fe.add_command(actuator, o80_pam::ActuatorState(pressure), speed, mode);
	   })

      .def("add_command",
	   [](frontend& fe,
	      int actuator,
	      int pressure,
	      o80::Duration_us duration,
	      o80::Mode mode) {
	     fe.add_command(actuator, o80_pam::ActuatorState(pressure), duration, mode);
	   })

      
      .def("add_command",
	   [](frontend& fe,
	      int actuator,
	      int pressure,
	      o80::Mode mode) {
	     fe.add_command(actuator, o80_pam::ActuatorState(pressure), mode);
	   })
      
        .def("add_command",
             [](frontend& fe,
                int dof,
                int ago,
                int antago,
                o80::Iteration it,
                o80::Mode mode) {
                 fe.add_command(2*dof, o80_pam::ActuatorState(ago), it, mode);
                 fe.add_command(2*dof+1, o80_pam::ActuatorState(antago), it, mode);
             })
        .def("add_command",
             [](frontend& fe,
                int dof,
                int ago,
                int antago,
                o80::Duration_us d,
                o80::Mode mode) {
                 fe.add_command(2*dof, o80_pam::ActuatorState(ago), d, mode);
                 fe.add_command(2*dof+1, o80_pam::ActuatorState(antago), d, mode);
             })
        .def("add_command",
             [](frontend& fe,
                int dof,
                int ago,
                int antago,
                o80::Speed s,
                o80::Mode mode) {
                 fe.add_command(2*dof, o80_pam::ActuatorState(ago), s, mode);
                 fe.add_command(2*dof+1, o80_pam::ActuatorState(antago), s, mode);
             })
        .def("add_command",
             [](frontend& fe, int dof, int ago, int antago, o80::Mode mode) {
                 fe.add_command(2*dof, o80_pam::ActuatorState(ago), mode);
                 fe.add_command(2*dof+1, o80_pam::ActuatorState(antago), mode);
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Speed s,
                o80::Mode mode) {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2*dof, o80_pam::ActuatorState(ago[dof]), s, mode);
                     fe.add_command(
                         2*dof+1, o80_pam::ActuatorState(antago[dof]), s, mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Iteration it,
                o80::Mode mode) {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2*dof, o80_pam::ActuatorState(ago[dof]), it, mode);
                     fe.add_command(
                         2*dof+1, o80_pam::ActuatorState(antago[dof]), it, mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Duration_us d,
                o80::Mode mode) {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2*dof, o80_pam::ActuatorState(ago[dof]), d, mode);
                     fe.add_command(
                         2*dof+1, o80_pam::ActuatorState(antago[dof]), d, mode);
                 }
             })
        .def("add_command",
             [](frontend& fe,
                std::array<int, NB_DOFS>& ago,
                std::array<int, NB_DOFS>& antago,
                o80::Mode mode) {
                 for (uint dof = 0; dof < NB_DOFS; dof++)
                 {
                     fe.add_command(
                         2*dof, o80_pam::ActuatorState(ago[dof]), mode);
                     fe.add_command(
                         2*dof+1, o80_pam::ActuatorState(antago[dof]), mode);
                 }
             });

}

PYBIND11_MODULE(o80_pam, m)
{

  // core bindings, common to dummy and real
  o80::Pybind11Config core_config;
  core_config.extended_state =
    false;                   // RobotState, already binded by pam_interface
  core_config.observation = false;  // added below
  core_config.frontend = false;     // added below
  o80::create_core_python_bindings<QUEUE_SIZE,
				   2*NB_DOFS,
				   ActuatorState,
				   RobotState>(m,core_config);
  add_observation(m);
  add_frontend(m);
  
  
  // wrappers for dummy robot
  o80::Pybind11Config dummy_config(true);
  dummy_config.prefix = std::string("dummy_");
  o80::create_python_bindings<DummyRobotDriver,
			      DummyStandalone,
			      pam_interface::Configuration<NB_DOFS>>(m,
								     dummy_config);

  // wrappers for real robot
  o80::Pybind11Config real_config(true);
  real_config.prefix = std::string("real_");
  o80::create_python_bindings<RealRobotDriver,
			      RealStandalone,
			      pam_interface::Configuration<NB_DOFS>>(m,
								     real_config);


  
}
