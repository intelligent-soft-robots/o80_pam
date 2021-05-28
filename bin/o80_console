#!/usr/bin/env python3

import sys
import time
import curses
import o80
import o80_pam
import threading
from lightargs import BrightArgs,Set,Range,Positive


class _Observation:

    def __init__(self,observation):
        self.iteration = observation.get_iteration()
        self.frequency = observation.get_frequency()
        self.time_stamp = observation.get_time_stamp()
        self.observed = observation.get_observed_pressures()
        self.desired = observation.get_desired_pressures()
        self.ref_founds = observation.get_references_found()
        self.positions = observation.get_positions()


def _get_status(frontend,config):
    observation_ = frontend.latest()
    observation = _Observation(observation_)
    positions = [0]*len(observation.ref_founds)
    for dof,found in enumerate(observation.ref_founds):
        if not found:
            positions[dof]=None
    s = ["iteration: {} | frequency {:.2f} | time stamp {}\n".format(observation.iteration,
                                                                     observation.frequency,
                                                                     observation.time_stamp)]
    for dof in range(config.nb_dofs):
        d = [dof]
        for ago in [0,1]:
            d.append([observation.observed[dof][ago],
                      observation.desired[dof][ago]])
        if not observation.ref_founds[dof]:
            d.append("*")
        else:
            position = observation.positions[dof]
            if position is None:
                d.append("*")
            else :
                d.append(position)
        d = "\t".join([str(d_) for d_ in d])
        s.append(d)
    return "\t"+"\n\t".join(s),observation.observed


class _Console:

    # manages curses : creates the o80 frontend that is
    # used to extract observation from the robot, and
    # display related selected information in the terminal
    
    _nb_dofs = None
    _screen = None
    _should_exit = False
    _monitor_exit_thread = None
    _permanent_info = None
    _min_pressure = None
    _max_pressure = None
    _width = None
    _frontend = None


    @classmethod
    def init(cls,
             permanent_info,
             config,frontend):
        cls._frontend = frontend
        cls._nb_dofs = config.nb_dofs
        cls._permanent_info = permanent_info
        cls._width = config.width
        cls._min_pressure = config.minimal_pressure
        cls._max_pressure = config.maximal_pressure
        cls._screen = curses.initscr()
        curses.noecho()
        curses.curs_set(0)
        cls._screen.keypad(1)
        cls._monitor_exit_thread = threading.Thread(target=cls._monitor_exit)
        cls._monitor_exit_thread.setDaemon(True)
        cls._monitor_exit_thread.start()

    @classmethod
    def _monitor_exit(cls):
        # detecting if 'q' was pressed, which set _should_exit
        # to True, which will stop the loop
        while not cls._should_exit:
            event = cls._screen.getch()
            if event == ord("q"):
                cls._should_exit=True

    @classmethod
    def exit(cls):
        # closing curses
        curses.endwin()

    @classmethod
    def _get_dof_str(cls,
                     dof,
                     pressure_plus,
                     pressure_minus):
        # the method that generates the "pressure bars"
        # for each of the dof
        def _get_pressure_str(pressure,
                              min_pressure,
                              max_pressure,
                              width):
            mdiff = (max_pressure-min_pressure)
            diff = pressure-min_pressure
            value = int (width*(diff)/mdiff)
            return '|'*value
        r = []
        for sign,pressure in zip(('+','-'),(pressure_plus,pressure_minus)):
            value = _get_pressure_str( pressure,
                                       cls._min_pressure,
                                       cls._max_pressure,
                                       cls._width) 
            r.append("\t{}{}  {}".format(dof,sign,value))
        return '\n'.join(r)


    @classmethod
    def refresh(cls,config):
        # The method the construct and display the
        # string information
        if cls._should_exit:
            return
        status,pressures  = _get_status(cls._frontend,config)
        cls._screen.clear()
        cls._screen.addstr(cls._permanent_info+"\n")
        # the "pressure bars"
        for dof in range(config.nb_dofs):
            cls._screen.addstr( cls._get_dof_str(dof,
                                                 pressures[dof][0],
                                                 pressures[dof][1]) )
            cls._screen.addstr('\n\n')
        # the status (iteration, frequency, observed/desired pressues)
        cls._screen.addstr(status)
        try :
            cls._screen.refresh()
        except:
            print("\n\n\nFAILED CONSOLE\n|{}|\n\n\n".format(status))


def _permanent_info(config):
    permanent_info = str("\n\to80 pam | segment_id: {}"+
                         "\n\t(press q to exit)\n").format(config.segment_id)
    return permanent_info


def _console(config,frontend):
    # Create a frontend on the segment_id, get observations
    # and displays important information on the terminal.
    _Console.init(_permanent_info(config),
                  config,frontend)
    while not _Console._should_exit:
        _Console.refresh(config)
        time.sleep(0.05)
    _Console.exit()


def _get_frontend(config):
    try :
        frontend = o80_pam.FrontEnd(config.segment_id)
        return frontend
    except Exception as e :
        return None

    
def _configure():
    config = BrightArgs("o80 PAM console")
    config.add_option("segment_id",
                      o80_pam.segment_ids.robot,
                      "o80 segment_id of the instance of o80 pam to display",
                      str)
    config.add_option("nb_dofs",
                      4,
                      "number of degrees of freedom of the robots",
                      int,
                      integrity_checks=[Positive()])
    config.add_option("refresh frequency",
                      100,
                      "refresh rate of the console (Hz)",
                      int,
                      [Range(1,1000)])
    config.add_option("minimal_pressure",
                      5000,
                      "minimal possible muscle pressure, for scaling purposes",
                      int,
                      [Positive()])
    config.add_option("maximal_pressure",
                      25000,
                      "maximal possible muscle pressure, for scaling purposes",
                      int,
                      [Positive()])
    config.add_option("width",
                      100,
                      "number of characters used for pressure bars",
                      int,
                      [Positive()])
    change_all=False
    finished  = config.dialog(change_all,sys.argv[1:])
    if not finished:
        return None
    return config


def execute():

    config = _configure()
    if config is None:
        return
    frontend = _get_frontend(config)
    if frontend is None:
        print(str("\nfailed to start an o80 frontend on segment_id: {}."+
                  "Is a corresponding robot running ?\n").format(config.segment_id))
    else:
        _console(config,frontend)

    
if __name__ == "__main__":

    execute()
