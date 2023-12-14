#!/usr/bin/env python3

import logging
import sys
import time
import math
import o80
import o80_pam
import matplotlib.pyplot as plt
from lightargs import BrightArgs, Set, Range, Positive


def plot(axis, data, dof, sign):
    # plot data (data as returned
    # by reach target above)
    if sign == 0:
        print("plotting for dof", dof, "agonist")
        ago = "agonist"
    else:
        print("plotting for dof", dof, "antagonist")
        ago = "antagonist"
    axis.set_title(str(dof) + " | " + ago)
    observed = [d.get_observed_pressures()[dof][sign] for d in data]
    desired = [d.get_desired_pressures()[dof][sign] for d in data]
    iterations = list(range(len(data)))
    axis.plot(iterations, observed, c="blue")
    axis.plot(iterations, desired, c="red")


def run(config, frontend):
    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format="[o80 check segment_id: {}] %(message)s".format(config.segment_id),
        level=logging.DEBUG,
        handlers=[log_handler],
    )

    speed = o80.Speed.per_second(config.speed)
    pressure_delta = config.pressure_delta
    all_data = {dof: [None, None] for dof in range(config.nb_dofs)}
    dofs = [dof for dof in range(config.nb_dofs)]

    for dof in dofs:
        logging.info("checking dof: {}".format(dof))
        start_iteration = frontend.latest().get_iteration()
        starting_pressures = frontend.latest().get_observed_pressures()[dof]
        frontend.add_command(2 * dof, starting_pressures[0], o80.Mode.QUEUE)
        frontend.add_command(
            2 * dof, starting_pressures[0] + pressure_delta, speed, o80.Mode.QUEUE
        )
        frontend.add_command(2 * dof, starting_pressures[0], speed, o80.Mode.QUEUE)
        frontend.pulse_and_wait()
        data = frontend.get_observations_since(start_iteration)
        all_data[dof][0] = data
        start_iteration = frontend.latest().get_iteration()
        starting_pressures = frontend.latest().get_observed_pressures()[dof]
        frontend.add_command(2 * dof + 1, starting_pressures[1], o80.Mode.QUEUE)
        frontend.add_command(
            2 * dof + 1, starting_pressures[1] + pressure_delta, speed, o80.Mode.QUEUE
        )
        frontend.add_command(2 * dof + 1, starting_pressures[1], speed, o80.Mode.QUEUE)
        frontend.pulse_and_wait()
        data = frontend.get_observations_since(start_iteration)
        all_data[dof][1] = data

    # plotting
    logging.info("plotting")
    fix, axes = plt.subplots(4, 2)
    for dof in dofs:
        for sign in range(2):
            axis = axes[dof, sign]
            data = all_data[dof][sign]
            plot(axis, data, dof, sign)
    plt.show()


def _get_frontend(config):
    try:
        frontend = o80_pam.FrontEnd(config.segment_id)
        return frontend
    except Exception as e:
        return None


def _configure():
    config = BrightArgs("o80 PAM console")
    config.add_option(
        "segment_id",
        o80_pam.segment_ids.robot,
        "o80 segment_id of the instance of o80 pam to display",
        str,
    )
    config.add_option(
        "nb_dofs",
        4,
        "number of degrees of freedom of the robots",
        int,
        integrity_checks=[Positive()],
    )
    config.add_option(
        "pressure_delta",
        2000,
        "each muscle will apply pressures +/- this value",
        int,
        [Range(1, 10000)],
    )
    config.add_option(
        "speed", 1000, "pressure of unit per seconds", int, [Range(1, 10000)]
    )
    change_all = False
    finished = config.dialog(change_all, sys.argv[1:])
    if not finished:
        return None
    return config


def execute():
    config = _configure()
    if config is None:
        return
    frontend = _get_frontend(config)
    if frontend is None:
        print(
            str(
                "\nfailed to start an o80 frontend on segment_id: {}."
                + "Is a corresponding robot running ?\n"
            ).format(config.segment_id)
        )
    else:
        print()
        run(config, frontend)
        print()


if __name__ == "__main__":
    execute()
