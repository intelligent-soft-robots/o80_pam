#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include "o80/standalone.hpp"
#include "o80_pam/dummy_robot.hpp"
#include "o80_pam/extended_state.hpp"
#include "o80_pam/pam_standalone.hpp"
#include "pam_interface/driver.hpp"
#include "pam_interface/dummy_interface.hpp"

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
#define FREQUENCY 1000

typedef pam_interface::DummyInterface<NB_DOFS> Interface;
typedef std::shared_ptr<Interface> InterfacePtr;

std::atomic<bool> RUNNING(true);
void stop(int)
{
    RUNNING = false;
    std::cout << "dummy standalone: ctr+c signal detected\n";
}

void run(int id, bool simulation)
{
    std::string segment_id(SEGMENT_ID);
    segment_id = segment_id + std::string("_") + std::to_string(id);

    o80::clear_shared_memory(segment_id);

    std::array<int, NB_DOFS> min_pressures;
    std::array<int, NB_DOFS> max_pressures;
    for (unsigned int i = 0; i < NB_DOFS; i++)
    {
        min_pressures[i] = MIN_PRESSURE;
        max_pressures[i] = MAX_PRESSURE;
    }

    InterfacePtr interface_ptr(
        new Interface(o80::Microseconds(CONTROL_PERIOD_US),
                      o80::Microseconds(SENSOR_PERIOD_US),
                      min_pressures,
                      max_pressures,
                      min_pressures,
                      max_pressures));

    pam_interface::Driver<2 * NB_DOFS> ri_driver(interface_ptr);

    // 2*NB_DOFS : 2 muscles per dof
    o80_pam::PamStandalone<QUEUE_SIZE, 2 * NB_DOFS> pam_standalone(
        ri_driver,
        MAX_ACTION_DURATION_S,
        MAX_INTER_ACTION_DURATION_S,
        FREQUENCY,
        segment_id,
        OBJECT_ID,
        simulation);

    bool running = true;

    pam_interface::PamRobotState<NB_DOFS> extended_state;

    pam_standalone.start();

    while (running && RUNNING)
    {
        if (simulation)
        {
            running = pam_standalone.spin(extended_state, FREQUENCY, true);
        }
        else
        {
            running = pam_standalone.spin(extended_state);
        }
    }

    pam_standalone.stop();
}

int main(int argc, char *argv[])
{
    int id;
    bool simulation;
    if (argc > 2)
    {
        id = atoi(argv[1]);
        int sim = atoi(argv[2]);
        if (sim > 0)
        {
            simulation = true;
        }
        else
        {
            simulation = false;
        }
    }
    else
    {
        std::cout << "\nusage: dummy_standalone id simulation\n";
        return 1;
    }

    // running the server until ctrl+c
    struct sigaction stopping;
    stopping.sa_handler = stop;
    sigemptyset(&stopping.sa_mask);
    stopping.sa_flags = 0;
    sigaction(SIGINT, &stopping, nullptr);
    RUNNING = true;
    int c = 0;

    run(id, simulation);
}
