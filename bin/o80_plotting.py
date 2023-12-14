#!/usr/bin/env python3

import sys
import os
import time
import threading
import multiprocessing
import signal_handler
import o80
import o80_pam
import pam_interface
from lightargs import BrightArgs, Set, Range, Positive


def _plot(config, frontend):
    from fyplot import dict_plot

    # creates 5 dynamic plots: 1 per dof (observed/desired pressure
    # for agonist and antagonist muscles + position) + frequency
    # uses fyplot (pip3 install fyplot)

    plot_config = dict_plot.Config()

    # 1 subplot per dof
    plot_config.channels = [str(dof) for dof in range(config.nb_dofs)]
    # 1 subplot for frequency
    plot_config.channels.append("frequency")

    # for the dof plots: y limit of pressures between
    # 0 and 30000
    lim_min = config.minimal_pressure
    lim_max = config.maximal_pressure
    plot_config.limits = {str(dof): (lim_min, lim_max) for dof in range(config.nb_dofs)}

    # for frequency: y limits in Hz up to 2x
    # the expected frequency
    plot_config.limits["frequency"] = (0, config.maximal_frequency)

    # each dof subplot will displays these pressures + position:
    # (position, in radian, will be cast to (min_pressure,max_pressure)
    # to fit in the y limits
    for dof in range(config.nb_dofs):
        plot_config.slots[str(dof)] = (
            "desired_ago",
            "observed_ago",
            "desired_antago",
            "observed_antago",
            "position",
        )

    # the frequency subplot
    plot_config.slots["frequency"] = ("frequency",)

    # colors. Ago: red, Antago: green
    #         observed: strong color, desired: ligher color
    #         position : some kind of pink
    #         frequency: blue
    plot_config.slots_colors = {}
    plot_config.slots_colors["desired_ago"] = (150, 0, 0)
    plot_config.slots_colors["observed_ago"] = (255, 0, 0)
    plot_config.slots_colors["desired_antago"] = (0, 150, 0)
    plot_config.slots_colors["observed_antago"] = (0, 255, 0)
    plot_config.slots_colors["position"] = (255, 100, 242)
    plot_config.slots_colors["frequency"] = (0, 255, 0)

    # x axis: how many data point
    plot_config.data_size = config.data_size
    plot_config.title = config.segment_id
    plot_config.windows_size = (config.window_width, config.window_height)

    # creates a o80 frontend, and uses it to get observation
    # and "feeding" the plots
    def run():
        class Holder:
            previous_iteration = None
            data = {}
            dofs = [str(dof) for dof in range(config.nb_dofs)]

        # position is in degrees between -180 and +180.
        # casting it to [lim_min,lim_max]
        def norm_position(position, lim_min=0, lim_max=30000):
            p = float(position) / 360.0
            diff = float(lim_max - lim_min)
            mid = diff / 2.0
            p = mid + p * diff
            return int(p + 0.5)

        def update_data():
            observation = frontend.latest()
            iteration = observation.get_iteration()
            if iteration == Holder.previous_iteration:
                Holder.data = {}
            Holder.previous_iteration = iteration
            observed_pressures = observation.get_observed_pressures()
            desired_pressures = observation.get_desired_pressures()
            positions = observation.get_positions()
            frequency = observation.get_frequency()
            for dof, observed, desired, position in zip(
                Holder.dofs, observed_pressures, desired_pressures, positions
            ):
                Holder.data[dof] = {
                    "desired_ago": desired[0],
                    "desired_antago": desired[1],
                    "observed_ago": observed[0],
                    "observed_antago": observed[1],
                }
                if position is not None:
                    Holder.data[dof]["position"] = norm_position(position)
            Holder.data["frequency"] = {"frequency": frequency}

        signal_handler.init()

        try:
            while not signal_handler.has_received_sigint():
                update_data()
                if Holder.data:
                    dict_plot.set_data(Holder.data)
                time.sleep(0.01)
        except KeyboardInterrupt:
            return

    dict_plot.start_plotting(plot_config, run)


def _configure():
    config = BrightArgs("o80 PAM console")
    config.add_option(
        "segment_id",
        o80_pam.segment_ids.robot,
        "o80 segment_id of the instance of o80 pam to display",
        str,
    )
    config.add_option(
        "maximal_frequency",
        3000,
        "maximal frequency of the o80 backend, for scaling",
        float,
        [Positive()],
    )
    config.add_option(
        "nb_dofs", 4, "number of degrees of freedom of the robot", int, [Positive()]
    )
    config.add_option("window_width", 1200, "plot window's width", int, [Positive()])
    config.add_option("window_height", 1000, "plot window's height", int, [Positive()])
    config.add_option(
        "data_size", 2000, "number of running data points per plot", int, [Positive()]
    )
    config.add_option(
        "minimal_pressure",
        5000,
        "minimal possible muscle pressure, for scaling purposes",
        int,
        [Positive()],
    )
    config.add_option(
        "maximal_pressure",
        25000,
        "maximal possible muscle pressure, for scaling purposes",
        int,
        [Positive()],
    )
    change_all = False
    finished = config.dialog(change_all, sys.argv[1:])
    if not finished:
        return None
    return config


def _get_frontend(config):
    try:
        frontend = o80_pam.FrontEnd(config.segment_id)
        return frontend
    except Exception as e:
        return None


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
        print(
            str(
                "\nplotting for segment_id: {}. "
                + "To exit: close the plot and press ctrl+c\n"
            ).format(config.segment_id)
        )

        _plot(config, frontend)


if __name__ == "__main__":
    execute()
