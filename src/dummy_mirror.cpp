#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include "o80/front_end.hpp"
#include "o80_pam/dummy_robot.hpp"
#include "o80_pam/o8o_pam_actuator_state.hpp"
#include "pam_interface/pam_robot_state.hpp"

#define NB_DOFS DUMMY_PAM_NB_DOFS
#define QUEUE_SIZE DUMMY_PAM_QUEUE_SIZE
#define SEGMENT_ID DUMMY_PAM_SEGMENT_ID
#define OBJECT_ID DUMMY_PAM_OBJECT_ID

typedef o80::FrontEnd<QUEUE_SIZE,
                      NB_DOFS * 2,
                      o80_pam::o80PamActuatorState,
                      pam_interface::PamRobotState<NB_DOFS> >
    Frontend;
typedef o80::Observation<NB_DOFS * 2,
                         o80_pam::o80PamActuatorState,
                         pam_interface::PamRobotState<NB_DOFS> >
    Observation;
typedef pam_interface::PamRobotState<NB_DOFS> RobotState;

std::atomic<bool> RUNNING(true);
void stop(int)
{
    RUNNING = false;
    std::cout << "dummy mirror: ctr+c signal detected\n";
}

int run(int from, int to)
{
    std::string segment_id_from =
        SEGMENT_ID + std::string("_") + std::to_string(from);
    Frontend frontend_from(segment_id_from, OBJECT_ID, false);

    std::string segment_id_to =
        SEGMENT_ID + std::string("_") + std::to_string(to);
    Frontend frontend_to(segment_id_to, OBJECT_ID, false);

    int previous_iteration = -1;
    int from_iteration = -1;

    while (RUNNING)
    {
        // waiting for origin robot to iterate at least once
        while (RUNNING && from_iteration == previous_iteration)
        {
            from_iteration = frontend_from.read().get_iteration();
            usleep(1);
        }
        previous_iteration = from_iteration;

        // creating command for setting mirrored robot
        // to same state as origin robot
        Observation observation = frontend_from.read();
        RobotState state = observation.get_extended_state();
        for (int dof = 0; dof < NB_DOFS; dof++)
        {
            int agonist = state.get(dof, pam_interface::Sign::AGONIST);
            int antagonist = state.get(dof, pam_interface::Sign::ANTAGONIST);
            frontend_to.add_command(2 * dof, agonist, o80::Mode::OVERWRITE);
            frontend_to.add_command(
                2 * dof + 1, antagonist, o80::Mode::OVERWRITE);
        }

        // sending commands to mirrored robot
        frontend_to.pulse();
    }
}

int main(int argc, char *argv[])
{
    int id_from = atoi(argv[1]);
    int id_to = atoi(argv[2]);

    // running the server until ctrl+c
    struct sigaction stopping;
    stopping.sa_handler = stop;
    sigemptyset(&stopping.sa_mask);
    stopping.sa_flags = 0;
    sigaction(SIGINT, &stopping, nullptr);
    RUNNING = true;
    int c = 0;

    run(id_from, id_to);
}
