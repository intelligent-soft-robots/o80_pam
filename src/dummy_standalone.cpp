#include <signal.h>
#include <unistd.h>
#include "shared_memory/serializer.hpp"
#include "real_time_tools/spinner.hpp"
#include "O8O/standalone.hpp"
#include "pam_interface/driver.hpp"
#include "pam_interface/dummy_interface.hpp"
#include "O8O_pam/pam_standalone.hpp"
#include "O8O_pam/extended_state.hpp"
#include "O8O_pam/dummy_robot.hpp"

#define NB_DOFS DUMMY_PAM_NB_DOFS
#define QUEUE_SIZE DUMMY_PAM_QUEUE_SIZE
#define SEGMENT_ID DUMMY_PAM_SEGMENT_ID
#define OBJECT_ID DUMMY_PAM_OBJECT_ID
#define CONTROL_PERIOD_US 200
#define SENSOR_PERIOD_US 200
#define MIN_PRESSURE 5000
#define MAX_PRESSURE 20000
#define MAX_ACTION_DURATION_S -1
#define MAX_INTER_ACTION_DURATION_S -1
#define FREQUENCY 100

typedef pam_interface::DummyInterface<NB_DOFS> Interface;
typedef std::shared_ptr<Interface> InterfacePtr;


std::atomic<bool> RUNNING(true);
void stop(int)
{
  RUNNING=false;
  std::cout << "dummy standalone: ctr+c signal detected\n";
}


void run()
{

  shared_memory::clear_shared_memory(SEGMENT_ID);
  shared_memory::clear_shared_memory(std::string(SEGMENT_ID)+std::string("_synchronizer"));
  shared_memory::clear_shared_memory(std::string(SEGMENT_ID)+std::string("_synchronizer_follower"));
  shared_memory::clear_shared_memory(std::string(SEGMENT_ID)+std::string("_synchronizer_leader"));
  
  std::array<int,NB_DOFS> min_pressures;
  std::array<int,NB_DOFS> max_pressures;
  for(unsigned int i=0;i<NB_DOFS;i++)
    {
      min_pressures[i]=MIN_PRESSURE;
      max_pressures[i]=MAX_PRESSURE;
    }

  InterfacePtr interface_ptr( new Interface ( O8O::Microseconds(CONTROL_PERIOD_US),
					      O8O::Microseconds(SENSOR_PERIOD_US),
					      min_pressures,
					      max_pressures,
					      min_pressures,
					      max_pressures) );

  pam_interface::Driver<2*NB_DOFS> ri_driver(interface_ptr);

  // 2*NB_DOFS : 2 muscles per dof
  O8O_pam::PamStandalone<QUEUE_SIZE,
			 2*NB_DOFS> pam_standalone(ri_driver,
						   MAX_ACTION_DURATION_S,
						   MAX_INTER_ACTION_DURATION_S,
						   FREQUENCY,
						   SEGMENT_ID,
						   OBJECT_ID);

  bool running = true;
  double observed_frequency = 0.0;

  pam_interface::PamRobotState<NB_DOFS> extended_state;

  pam_standalone.start();

  while(running && RUNNING)
    {

      running = pam_standalone.spin(extended_state,200,true);
      //running = pam_standalone.spin(extended_state);

    }
  
  pam_standalone.stop();
  
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
