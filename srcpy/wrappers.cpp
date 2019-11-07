#include "O8O/front_end.hpp"
#include "O8O_pam/o8o_pam_actuator_state.hpp"
#include "O8O_pam/dummy_robot.hpp"
#include "pam_interface/pam_robot_state.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>


#define NB_DOFS DUMMY_PAM_NB_DOFS
#define QUEUE_SIZE DUMMY_PAM_QUEUE_SIZE
#define SEGMENT_ID DUMMY_PAM_SEGMENT_ID
#define OBJECT_ID DUMMY_PAM_OBJECT_ID



typedef O8O::Observation< NB_DOFS*2,
			  O8O_pam::O8OPamActuatorState,
			  pam_interface::PamRobotState<NB_DOFS> > Observation;
typedef pam_interface::PamRobotState<NB_DOFS> RobotState;
typedef O8O_pam::O8OPamActuatorState ActuatorState;
typedef O8O::FrontEnd<QUEUE_SIZE,
		      NB_DOFS*2,
		      O8O_pam::O8OPamActuatorState,
		      pam_interface::PamRobotState<NB_DOFS> > Frontend;
typedef std::shared_ptr<Frontend> FrontendPtr;

static FrontendPtr FRONTEND;
static bool SIMULATION = false;


static FrontendPtr frontend()
{
  if(FRONTEND==nullptr)
    {
      FRONTEND.reset(new Frontend(SEGMENT_ID,
				  OBJECT_ID,
				  SIMULATION));
    }
  return FRONTEND;
}


PYBIND11_MODULE(pam_O8O,m){

  pybind11::class_<O8O::Iteration>(m,"Iteration")
    .def(pybind11::init<>())
    .def(pybind11::init<long int>());

  pybind11::enum_<O8O::Mode>(m,"Mode")
    .value("STACK",O8O::STACK)
    .value("OVERWRITE",O8O::OVERWRITE);

  pybind11::enum_<pam_interface::Sign>(m,"Sign")
    .value("AGONIST",pam_interface::Sign::AGONIST)
    .value("ANTAGONIST",pam_interface::Sign::ANTAGONIST);
  
  pybind11::class_<RobotState>(m,"RobotState")
    .def(pybind11::init<>())
    .def("get",&RobotState::get)
    .def("get_desired",&RobotState::get_desired)
    .def("get_reference_found",&RobotState::get_reference_found)
    .def("get_position",&RobotState::get_position)
    .def("get_velocity",&RobotState::get_velocity);
    
  pybind11::class_<Observation>(m,"Observation")
    .def(pybind11::init<>())
    .def("get_iteration",&Observation::get_iteration)
    .def("get_frequency",&Observation::get_frequency)
    .def("get",&Observation::get_extended_state);

  m.def("set_simulation",[]()
	{
	  SIMULATION = true;
	});
  
  m.def("execute_command",[](const std::map<int,int>& agonists,
			     const std::map<int,int>& antagonists,
			     int nb_iterations,
			     O8O::Mode mode)
	{
	  FrontendPtr fe = frontend();
	  int iteration_now = fe->read().get_iteration();
	  O8O::Iteration target_iteration(iteration_now+nb_iterations);
	  for(auto const& dof_pressure: agonists)
	    {
	      int dof = dof_pressure.first;
	      ActuatorState desired_state(dof_pressure.second);
	      int actuator = 2*dof;
	      fe->add_command(actuator,desired_state,target_iteration,mode);
	    }
	  for(auto const& dof_pressure: antagonists)
	    {
	      int dof = dof_pressure.first;
	      ActuatorState desired_state(dof_pressure.second);
	      int actuator = 2*dof+1;
	      fe->add_command(actuator,desired_state,target_iteration,mode);
	    }
	  return fe->pulse(target_iteration).get_extended_state();
	});

  m.def("status",[]()
	{
	  FrontendPtr fe = frontend();
	  Observation obs = fe->read();
	  pam_interface::PamRobotState<NB_DOFS> state = obs.get_extended_state();
	  std::cout << "Iteration\t" << obs.get_iteration() << "\n";
	  std::cout << "Frequency\t"<< obs.get_frequency() << "\n";
	  for(int dof=0;dof<NB_DOFS;dof++)
	    {
	      std::cout << "\tagonist:"
			<< state.get(dof,pam_interface::Sign::AGONIST)
			<< "\tantagonist:"
			<< state.get(dof,pam_interface::Sign::ANTAGONIST)
			<< "\treference found:"
			<< state.get_reference_found(dof)
			<< "\tposition:"
			<< state.get_position(dof)
			<< "\tvelocity:"
			<< state.get_velocity(dof)
			<< "\n";
	    }
	});
  
  m.def("add_command",[](int dof,
			 pam_interface::Sign sign,
			 int target_pressure,
			 long int target_iteration,
			 O8O::Mode mode)
	{
	  FrontendPtr fe = frontend();
	  O8O::Iteration iteration(target_iteration);
	  ActuatorState desired_state(target_pressure);
	  int actuator = 2*dof;
	  if(sign==pam_interface::Sign::ANTAGONIST)
	    {
	      actuator++;
	    }
	  fe->add_command(actuator,desired_state,iteration,mode);
	});


  m.def("iteration_wait",[](int target_iteration)
	{
	  FrontendPtr fe = frontend();
	  O8O::Iteration iteration(target_iteration);
	  Observation obs = fe->pulse(target_iteration);
	  return obs.get_extended_state();
	});

  m.def("pulse",[]()
	{
	  FrontendPtr fe = frontend();
	  return fe->pulse().get_extended_state();
	});

  m.def("wait_for_completion",[]()
	{
	  FrontendPtr fe = frontend();
	  return fe->pulse_and_wait().get_extended_state();
	});

  m.def("read",[]()
	{
	  FrontendPtr fe = frontend();
	  return fe->read();
	});

  m.def("last_iteration",[]()
	{
	  FrontendPtr fe = frontend();
	  return fe->read().get_iteration();
	});
  
  


}
