#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "O8O/front_end.hpp"
#include "O8O/pybind_helper.hpp"

#include "pam_interface/state/robot.hpp"
#include "pam_interface/pressure_action.hpp"
#include "pam_interface/dummy/driver.hpp"
#include "pam_interface/configuration.hpp"

#include "O8O_pam/actuator_state.hpp"
#include "O8O_pam/standalone.hpp"


#define NB_DOFS 4
#define QUEUE_SIZE 50000
#define SEGMENT_ID "dummy_pam"
#define OBJECT_ID "dummy_pam"


// pressures action sent to the robot_interfaces backend
typedef pam_interface::PressureAction<NB_DOFS*2> Action;

// pressures red from the robot interfaces backend
typedef pam_interface::RobotState<NB_DOFS> Observation;

// drivers used by the robot_interfaces backend
typedef pam_interface::DummyRobotDriver<NB_DOFS> DummyRobotDriver;

// used as argument to the driver
typedef pam_interface::Configuration<NB_DOFS> Configuration;

// state of each actuator that will be encapsulated
// in O8O::Observation
typedef O8O_pam::ActuatorState ActuatorState;

// O8O Standalone class 
typedef O8O_pam::Standalone<QUEUE_SIZE,
			    NB_DOFS*2,
			    DummyRobotDriver> Standalone;



PYBIND11_MODULE(pam_O8O,m){

  O8O::create_python_bindings<QUEUE_SIZE,
			      NB_DOFS*2,
			      Action,
			      Observation,
			      ActuatorState,
			      DummyRobotDriver,
			      Standalone,
			      O8O::EmptyExtendedState,
			      Configuration>(m);
  
}
