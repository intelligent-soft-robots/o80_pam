#include <signal.h>
#include <unistd.h>
#include "O8O/front_end.hpp"
#include "O8O_pam/o8o_pam_actuator_state.hpp"
#include "pam_interface/pam_robot_state.hpp"
#include "O8O_pam/dummy_robot.hpp"

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


std::atomic<bool> RUNNING(true);
void stop(int)
{
  RUNNING=false;
  std::cout << "dummy frontend: ctr+c signal detected\n";
}


int get_iteration(const Observation& observation)
{
  return observation.get_extended_state().get_update_iteration();
}

double get_frequency(const Observation& observation)
{
  return observation.get_extended_state().get_update_frequency();
}

int get_pressure(const Observation& observation)
{
  const RobotState& robot = observation.get_extended_state();
  return robot.get(0,pam_interface::Sign::AGONIST);
}

void run()
{

  Frontend frontend(SEGMENT_ID,
		    OBJECT_ID);




  Observation observation = frontend.pulse();
  int iteration = get_iteration(observation);

  ActuatorState desired_state(15000);
  frontend.add_command(0,
		       desired_state,
		       O8O::Iteration(iteration+50),
		       O8O::Mode::OVERWRITE);
  frontend.pulse_and_wait();

  
  
  
  while(RUNNING)
    {
      observation = frontend.pulse(iteration+10);
      iteration = get_iteration(observation);
      double frequency = get_frequency(observation);
      int pressure = get_pressure(observation);
      std::cout << iteration << "\t" << frequency << "\t" << pressure <<"\n";
    }

}


int main()
{
  // running the server until ctrl+c
  struct sigaction stopping;
  stopping.sa_handler = stop;
  sigemptyset(&stopping.sa_mask);
  stopping.sa_flags = 0;
  sigaction(SIGINT, &stopping, nullptr);
  RUNNING=true;
  int c = 0;

  run();
}
