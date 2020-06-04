#include <signal.h>
#include <unistd.h>
#include "o80/front_end.hpp"
#include "o80_pam/o8o_pam_actuator_state.hpp"
#include "pam_interface/pam_robot_state.hpp"
#include "o80_pam/dummy_robot.hpp"

#define NB_DOFS DUMMY_PAM_NB_DOFS
#define QUEUE_SIZE DUMMY_PAM_QUEUE_SIZE
#define SEGMENT_ID DUMMY_PAM_SEGMENT_ID
#define OBJECT_ID DUMMY_PAM_OBJECT_ID

typedef o80::Observation< NB_DOFS*2,
			  o80_pam::o80PamActuatorState,
			  pam_interface::PamRobotState<NB_DOFS> > Observation;
typedef pam_interface::PamRobotState<NB_DOFS> RobotState;
typedef o80_pam::o80PamActuatorState ActuatorState;
typedef o80::FrontEnd<QUEUE_SIZE,
		      NB_DOFS*2,
		      o80_pam::o80PamActuatorState,
		      pam_interface::PamRobotState<NB_DOFS> > Frontend;


std::atomic<bool> RUNNING(true);
void stop(int)
{
  RUNNING=false;
  std::cout << "dummy frontend: ctr+c signal detected\n";
}


int get_iteration(const Observation& observation)
{
  return observation.get_iteration();
}

double get_frequency(const Observation& observation)
{
  return observation.get_frequency();
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
		       o80::Iteration(iteration+50),
		       o80::Mode::OVERWRITE);
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
