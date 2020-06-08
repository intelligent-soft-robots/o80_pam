#! /usr/bin/env python

import math,copy,threading,curses,time
import o80
import o80_pam
import pam_interface

MIN_PRESSURE = 0
MAX_PRESSURE = 30000
NB_DOFS = 4

class robot_state_display:

    _nb_dofs = NB_DOFS
    _screen = None
    _should_exit = False
    _monitor_exit_thread = None
    _min_pressure = MIN_PRESSURE
    _max_pressure = MAX_PRESSURE
    _width = None
    _frontend = None
    
    @classmethod
    def init(cls,
             segment_id,
             width = 100):

        cls._frontend = o80_pam.FrontEnd(segment_id)
        
        # display config

        cls._width = width

        # init curses
        
        cls._screen = curses.initscr()
        curses.noecho()
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1,curses.COLOR_GREEN,-1)
        curses.init_pair(2,curses.COLOR_BLUE,-1)
        cls._screen.keypad(1)
        cls._monitor_exit_thread = threading.Thread(target=cls._monitor_exit)
        cls._monitor_exit_thread.setDaemon(True)
        cls._monitor_exit_thread.start()
        
    @classmethod
    def _monitor_exit(cls):
        while not cls._should_exit:
            event = cls._screen.getch()
            if event == ord("q"): cls._should_exit=True


    @classmethod
    def exit(cls):
        curses.endwin()

    @classmethod
    def _get_dof_str(cls,
                     dof,
                     pressure_plus,
                     pressure_minus):

        def _get_pressure_str(pressure,
                              min_pressure,
                              max_pressure,
                              width):

            value = int (width * (pressure-min_pressure)/(max_pressure-min_pressure) )
            return '|'*value

        r = []
        
        for sign,pressure in zip(('+','-'),(pressure_plus,pressure_minus)):

            value = _get_pressure_str( pressure,
                                       cls._min_pressure,
                                       cls._max_pressure,
                                       cls._width) 

            r.append(str(dof)+sign+"  "+str(value))

        return '\n'.join(r)
        
        
    @classmethod
    def refresh(cls):

        if cls._should_exit:
            return

        pressures = cls._frontend.latest().get_observed_pressures()
        
        cls._screen.clear()
        
        for dof in range(NB_DOFS):
            
            cls._screen.addstr( cls._get_dof_str(dof,
                                                 pressures[dof][0],
                                                 pressures[dof][1]) )
            cls._screen.addstr('\n\n')

        cls._screen.refresh()
            

        
if __name__ == "__main__":

    robot_state_display.init("o80_dummy_pam")

    while True :

        if robot_state_display._should_exit:
            break

        robot_state_display.refresh()
        
        time.sleep(0.05)

    robot_state_display.exit()
