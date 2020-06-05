#include <memory>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "o80/front_end.hpp"
#include "o80/pybind11_helper.hpp"

#include "pam_interface/state/robot.hpp"
#include "pam_interface/pressure_action.hpp"
#include "pam_interface/dummy/driver.hpp"
#include "pam_interface/configuration.hpp"

#include "o80_pam/actuator_state.hpp"
#include "o80_pam/standalone.hpp"


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
// in o80::Observation
typedef o80_pam::ActuatorState ActuatorState;

// o80 Standalone class 
typedef o80_pam::Standalone<QUEUE_SIZE,
			    NB_DOFS*2,
			    DummyRobotDriver> Standalone;



PYBIND11_MODULE(o80_pam,m){

  o80::Pybind11Config config;
  config.extended_state=false;
  
  o80::create_python_bindings<DummyRobotDriver,
			      Standalone,
			      pam_interface::Configuration<NB_DOFS>>(m,config);
  
}
