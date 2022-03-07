#!/usr/bin/env python3

from o80_pam import run_robot
        
if __name__ == "__main__":

    real_robot = False
    config = run_robot.configure("dummy")
    if config is not None:
        run_robot.run(config)

        
